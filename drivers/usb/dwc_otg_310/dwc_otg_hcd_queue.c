/* ==========================================================================
 * $File: //dwh/usb_iip/dev/software/otg/linux/drivers/dwc_otg_hcd_queue.c $
 * $Revision: #44 $
 * $Date: 2011/10/26 $
 * $Change: 1873028 $
 *
 * Synopsys HS OTG Linux Software Driver and documentation (hereinafter,
 * "Software") is an Unsupported proprietary work of Synopsys, Inc. unless
 * otherwise expressly agreed to in writing between Synopsys and you.
 *
 * The Software IS NOT an item of Licensed Software or Licensed Product under
 * any End User Software License Agreement or Agreement for Licensed Product
 * with Synopsys or any supplement thereto. You are permitted to use and
 * redistribute this Software in source and binary forms, with or without
 * modification, provided that redistributions of source code must retain this
 * notice. You may not view, use, disclose, copy or distribute this file or
 * any information contained herein except pursuant to this license grant from
 * Synopsys. If you do not agree with this notice, including the disclaimer
 * below, then you are not authorized to use the Software.
 *
 * THIS SOFTWARE IS BEING DISTRIBUTED BY SYNOPSYS SOLELY ON AN "AS IS" BASIS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE HEREBY DISCLAIMED. IN NO EVENT SHALL SYNOPSYS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 * ========================================================================== */
#ifndef DWC_DEVICE_ONLY

/**
 * @file
 *
 * This file contains the functions to manage Queue Heads and Queue
 * Transfer Descriptors.
 */

#include <linux/gcd.h>
#include "dwc_otg_hcd.h"
#include "dwc_otg_regs.h"

/* Wait this long before releasing periodic reservation */
#define DWC_OTG_HCD_UNRESERVE_DELAY (msecs_to_jiffies(5))

static void dwc_otg_hcd_uframe_unschedule(dwc_otg_hcd_t *hcd, dwc_otg_qh_t *qh);

static void dwc_otg_hcd_do_unreserve(dwc_otg_hcd_t *hcd, dwc_otg_qh_t *qh)
{
	assert_spin_locked((spinlock_t *)(hcd->lock));

	WARN_ON(!qh->unreserve_pending);

	/* No more unreserve pending--we're doing it */
	qh->unreserve_pending = false;

	if (WARN_ON(!DWC_LIST_EMPTY(&qh->qh_list_entry)))
		DWC_LIST_REMOVE_INIT(&qh->qh_list_entry);

	/* Update claimed usecs per (micro)frame */
	hcd->periodic_usecs -= qh->host_us;

	dwc_otg_hcd_uframe_unschedule(hcd, qh);
	/* Release periodic channel reservation */
	hcd->periodic_channels--;
}

static void dwc_otg_hcd_unreserve_timer_fn(unsigned long data)
{
	dwc_otg_qh_t *qh = (dwc_otg_qh_t *)data;
	dwc_otg_hcd_t *hcd = qh->hcd;
	unsigned long flags;

	/*
	 * Wait for the lock, or for us to be scheduled again.  We
	 * could be scheduled again if:
	 * - We started executing but didn't get the lock yet.
	 * - A new reservation came in, but cancel didn't take effect
	 *   because we already started executing.
	 * - The timer has been kicked again.
	 * In that case cancel and wait for the next call.
	 */
	while (!spin_trylock_irqsave((spinlock_t *)(hcd->lock), flags)) {
		if (timer_pending(&qh->unreserve_timer))
			return;
	}

	/*
	 * Might be no more unreserve pending if:
	 * - We started executing but didn't get the lock yet.
	 * - A new reservation came in, but cancel didn't take effect
	 *   because we already started executing.
	 *
	 * We can't put this in the loop above because unreserve_pending needs
	 * to be accessed under lock, so we can only check it once we got the
	 * lock.
	 */
	if (qh->unreserve_pending)
		dwc_otg_hcd_do_unreserve(hcd, qh);

	DWC_SPINUNLOCK_IRQRESTORE(hcd->lock, flags);
}

static struct dwc_otg_hcd_tt *dwc_otg_hcd_get_tt_info(dwc_otg_hcd_t *hcd, void *context,
						      int atomic_alloc, int *ttport)
{
	struct urb *urb = context;
	struct dwc_otg_hcd_tt *dwc_tt = NULL;

	if (urb->dev->tt) {
		*ttport = urb->dev->ttport;

		dwc_tt = urb->dev->tt->hcpriv;
		if (!dwc_tt) {
			size_t bitmap_size;

			/*
			 * For single_tt we need one schedule.  For multi_tt
			 * we need one per port.
			 */
			bitmap_size = DWC_OTG_HCD_ELEMENTS_PER_LS_BITMAP *
				sizeof(dwc_tt->periodic_bitmaps[0]);
			if (urb->dev->tt->multi)
				bitmap_size *= urb->dev->tt->hub->maxchild;

			if (atomic_alloc)
				dwc_tt = DWC_ALLOC_ATOMIC(sizeof(*dwc_tt) + bitmap_size);
			else
				dwc_tt = DWC_ALLOC(sizeof(*dwc_tt) + bitmap_size);

			if (!dwc_tt)
				return NULL;

			dwc_tt->usb_tt = urb->dev->tt;
			dwc_tt->usb_tt->hcpriv = dwc_tt;
		}

		dwc_tt->refcount++;
	}

	return dwc_tt;
}

static void dwc_otg_hcd_put_tt_info(dwc_otg_hcd_t *hcd, struct dwc_otg_hcd_tt *dwc_tt)
{
	/* Model kfree and make put of NULL a no-op */
	if (!dwc_tt)
		return;

	WARN_ON(dwc_tt->refcount < 1);

	dwc_tt->refcount--;
	if (!dwc_tt->refcount) {
		dwc_tt->usb_tt->hcpriv = NULL;
		DWC_FREE(dwc_tt);
	}
}

/**
 * Free each QTD in the QH's QTD-list then free the QH.  QH should already be
 * removed from a list.  QTD list should already be empty if called from URB
 * Dequeue.
 *
 * @param hcd HCD instance.
 * @param qh The QH to free.
 */
void dwc_otg_hcd_qh_free(dwc_otg_hcd_t *hcd, dwc_otg_qh_t *qh)
{
	dwc_otg_qtd_t *qtd, *qtd_tmp;
	dwc_irqflags_t flags;

	/* Free each QTD in the QTD list */
	DWC_SPINLOCK_IRQSAVE(hcd->lock, &flags);
	DWC_CIRCLEQ_FOREACH_SAFE(qtd, qtd_tmp, &qh->qtd_list, qtd_list_entry) {
		dwc_otg_hcd_qtd_remove_and_free(hcd, qtd, qh);
	}

	dwc_otg_hcd_put_tt_info(hcd, qh->dwc_tt);

	if (hcd->core_if->dma_desc_enable) {
		dwc_otg_hcd_qh_free_ddma(hcd, qh);
	} else if (qh->dw_align_buf) {
		uint32_t buf_size;
		if (qh->ep_type == UE_ISOCHRONOUS) {
			buf_size = 4096;
		} else {
			buf_size = hcd->core_if->core_params->max_transfer_size;
		}
		DWC_DMA_FREE(buf_size, qh->dw_align_buf, qh->dw_align_buf_dma);
	}
	DWC_SPINUNLOCK_IRQRESTORE(hcd->lock, flags);

	/* Make sure any unreserve work is finished. */
	if (del_timer_sync(&qh->unreserve_timer)) {
		DWC_SPINLOCK_IRQSAVE(hcd->lock, &flags);
		dwc_otg_hcd_do_unreserve(hcd, qh);
		DWC_SPINUNLOCK_IRQRESTORE(hcd->lock, flags);
	}

	DWC_FREE(qh);
	return;
}

/**
 * Initializes a QH structure.
 *
 * @param hcd The HCD state structure for the DWC OTG controller.
 * @param qh  The QH to init.
 * @param urb Holds the information about the device/endpoint that we need
 * 	      to initialize the QH.
 */
void qh_init(dwc_otg_hcd_t *hcd, dwc_otg_qh_t *qh, dwc_otg_hcd_urb_t *urb, int atomic_alloc)
{
	char *speed, *type;
	hprt0_data_t hprt;
	int dev_speed = hcd->fops->speed(hcd, urb->priv);
	uint8_t ep_type = dwc_otg_hcd_get_pipe_type(&urb->pipe_info);
	bool ep_is_in = !!dwc_otg_hcd_is_pipe_in(&urb->pipe_info);
	bool ep_is_isoc = (ep_type == UE_ISOCHRONOUS);
	bool ep_is_int = (ep_type == UE_INTERRUPT);
	int maxp = dwc_otg_hcd_get_mps(&urb->pipe_info);
	int bytecount = dwc_hb_mult(maxp) * dwc_max_packet(maxp);
	uint32_t prtspd;
	bool do_split;

	hprt.d32 = DWC_READ_REG32(hcd->core_if->host_if->hprt0);
	prtspd = hprt.b.prtspd;
	do_split = (prtspd == DWC_HPRT0_PRTSPD_HIGH_SPEED
		    && dev_speed != USB_SPEED_HIGH);

	dwc_memset(qh, 0, sizeof(dwc_otg_qh_t));

	/* Initialize QH */
	qh->hcd = hcd;
	setup_timer(&qh->unreserve_timer, dwc_otg_hcd_unreserve_timer_fn,
		    (unsigned long)qh);

	qh->ep_type = ep_type;
	qh->ep_is_in = ep_is_in;
	qh->data_toggle = DWC_OTG_HC_PID_DATA0;
	qh->maxp = maxp;
	DWC_CIRCLEQ_INIT(&qh->qtd_list);
	DWC_LIST_INIT(&qh->qh_list_entry);
	qh->channel = NULL;

	qh->do_split = do_split;
	qh->dev_speed = dev_speed;

	if (ep_is_int || ep_is_isoc) {
		/* Compute scheduling parameters once and save them */
		int host_speed = do_split ? USB_SPEED_HIGH : dev_speed;

		struct dwc_otg_hcd_tt *dwc_tt = dwc_otg_hcd_get_tt_info(hcd, urb->priv,
									atomic_alloc,
									&qh->ttport);
		int device_ns;

		qh->dwc_tt = dwc_tt;

		qh->host_us = NS_TO_US(usb_calc_bus_time(host_speed, ep_is_in,
							 ep_is_isoc, bytecount));
		device_ns = usb_calc_bus_time(dev_speed, ep_is_in,
					      ep_is_isoc, bytecount);

		if (do_split && dwc_tt)
			device_ns += dwc_tt->usb_tt->think_time;
		qh->device_us = NS_TO_US(device_ns);

		qh->device_interval = urb->interval;
		qh->host_interval = urb->interval * (do_split ? 8 : 1);

		/*
		 * Schedule low speed if we're running the host in low or
		 * full speed OR if we've got a "TT" to deal with to access this
		 * device.
		 */
		qh->schedule_low_speed = (prtspd != DWC_HPRT0_PRTSPD_HIGH_SPEED
					  || dwc_tt);

		if (do_split) {
			/* We won't know num transfers until we schedule */
			qh->num_hs_transfers = -1;
		} else if (dev_speed == USB_SPEED_HIGH) {
			qh->num_hs_transfers = 1;
		} else {
			qh->num_hs_transfers = 0;
		}

		/* We'll schedule later when we have something to do */
	}

	switch (dev_speed) {
	case USB_SPEED_LOW:
		speed = "low";
		break;
	case USB_SPEED_FULL:
		speed = "full";
		break;
	case USB_SPEED_HIGH:
		speed = "high";
		break;
	default:
		speed = "?";
		break;
	}

	switch (qh->ep_type) {
	case UE_ISOCHRONOUS:
		type = "isochronous";
		break;
	case UE_INTERRUPT:
		type = "interrupt";
		break;
	case UE_CONTROL:
		type = "control";
		break;
	case UE_BULK:
		type = "bulk";
		break;
	default:
		type = "?";
		break;
	}

	DWC_DEBUGPL(DBG_HCDV, "QH=%p Init %s, %s speed, %d bytes:\n", qh, type,
		    speed, bytecount);
	DWC_DEBUGPL(DBG_HCDV, "QH=%p ...addr=%d, ep=%d, %s\n", qh,
		    dwc_otg_hcd_get_dev_addr(&urb->pipe_info),
		    dwc_otg_hcd_get_ep_num(&urb->pipe_info),
		    ep_is_in ? "IN" : "OUT");
	if (ep_is_int || ep_is_isoc) {
		DWC_DEBUGPL(DBG_HCDV,
			    "QH=%p ...duration: host=%d us, device=%d us\n",
			    qh, qh->host_us, qh->device_us);
		DWC_DEBUGPL(DBG_HCDV,"QH=%p ...interval: host=%d, device=%d\n",
			    qh, qh->host_interval, qh->device_interval);
		if (qh->schedule_low_speed)
			DWC_DEBUGPL(DBG_HCDV, "QH=%p ...low speed schedule=%p\n",
				    qh, dwc_otg_hcd_get_ls_map(hsotg, qh));
	}
}

/**
 * This function allocates and initializes a QH.
 *
 * @param hcd The HCD state structure for the DWC OTG controller.
 * @param urb Holds the information about the device/endpoint that we need
 * 	      to initialize the QH.
 * @param atomic_alloc Flag to do atomic allocation if needed
 *
 * @return Returns pointer to the newly allocated QH, or NULL on error. */
dwc_otg_qh_t *dwc_otg_hcd_qh_create(dwc_otg_hcd_t *hcd,
				    dwc_otg_hcd_urb_t *urb, int atomic_alloc)
{
	dwc_otg_qh_t *qh;

	if (!urb->priv)
		return NULL;

	/* Allocate memory */
	/** @todo add memflags argument */
	qh = dwc_otg_hcd_qh_alloc(atomic_alloc);
	if (qh == NULL) {
		DWC_ERROR("qh allocation failed");
		return NULL;
	}

	qh_init(hcd, qh, urb, atomic_alloc);

	if (hcd->core_if->dma_desc_enable
	    && (dwc_otg_hcd_qh_init_ddma(hcd, qh) < 0)) {
		dwc_otg_hcd_qh_free(hcd, qh);
		return NULL;
	}

	return qh;
}

/**
 * Checks that a channel is available for a periodic transfer.
 *
 * @return 0 if successful, negative error code otherise.
 */
static int periodic_channel_available(dwc_otg_hcd_t *hcd)
{
	/*
	 * Currently assuming that there is a dedicated host channnel for each
	 * periodic transaction plus at least one host channel for
	 * non-periodic transactions.
	 */
	int status;
	int num_channels;

	num_channels = hcd->core_if->core_params->host_channels;
	if ((hcd->periodic_channels + hcd->non_periodic_channels < num_channels)
	    && (hcd->periodic_channels < num_channels - 1)) {
		status = 0;
	} else {
		DWC_INFO("%s: Total channels: %d, Periodic: %d, Non-periodic: %d\n",
			 __func__, num_channels, hcd->periodic_channels,
			 hcd->non_periodic_channels);
		status = -DWC_E_NO_SPACE;
	}

	return status;
}

/**
 * Checks that the max transfer size allowed in a host channel is large enough
 * to handle the maximum data transfer in a single (micro)frame for a periodic
 * transfer.
 *
 * @param hcd The HCD state structure for the DWC OTG controller.
 * @param qh QH for a periodic endpoint.
 *
 * @return 0 if successful, negative error code otherwise.
 */
static int check_max_xfer_size(dwc_otg_hcd_t *hcd, dwc_otg_qh_t *qh)
{
	int status;
	uint32_t max_xfer_size;
	uint32_t max_channel_xfer_size;

	status = 0;

	max_xfer_size = dwc_max_packet(qh->maxp) * dwc_hb_mult(qh->maxp);
	max_channel_xfer_size = hcd->core_if->core_params->max_transfer_size;

	if (max_xfer_size > max_channel_xfer_size) {
		DWC_INFO("%s: Periodic xfer length %d > " "max xfer length for channel %d\n",
			 __func__, max_xfer_size, max_channel_xfer_size);
		status = -DWC_E_NO_SPACE;
	}

	return status;
}

/**
 * pmap_schedule() - Schedule time in a periodic bitmap (pmap).
 *
 * @map:             The bitmap representing the schedule; will be updated
 *                   upon success.
 * @bits_per_period: The schedule represents several periods.  This is how many
 *                   bits are in each period.  It's assumed that the beginning
 *                   of the schedule will repeat after its end.
 * @periods_in_map:  The number of periods in the schedule.
 * @num_bits:        The number of bits we need per period we want to reserve
 *                   in this function call.
 * @interval:        How often we need to be scheduled for the reservation this
 *                   time.  1 means every period.  2 means every other period.
 *                   ...you get the picture?
 * @start:           The bit number to start at.  Normally 0.  Must be within
 *                   the interval or we return failure right away.
 * @only_one_period: Normally we'll allow picking a start anywhere within the
 *                   first interval, since we can still make all repetition
 *                   requirements by doing that.  However, if you pass true
 *                   here then we'll return failure if we can't fit within
 *                   the period that "start" is in.
 *
 * The idea here is that we want to schedule time for repeating events that all
 * want the same resource.  The resource is divided into fixed-sized periods
 * and the events want to repeat every "interval" periods.  The schedule
 * granularity is one bit.
 *
 * To keep things "simple", we'll represent our schedule with a bitmap that
 * contains a fixed number of periods.  This gets rid of a lot of complexity
 * but does mean that we need to handle things specially (and non-ideally) if
 * the number of the periods in the schedule doesn't match well with the
 * intervals that we're trying to schedule.
 *
 * Here's an explanation of the scheme we'll implement, assuming 8 periods.
 * - If interval is 1, we need to take up space in each of the 8
 *   periods we're scheduling.  Easy.
 * - If interval is 2, we need to take up space in half of the
 *   periods.  Again, easy.
 * - If interval is 3, we actually need to fall back to interval 1.
 *   Why?  Because we might need time in any period.  AKA for the
 *   first 8 periods, we'll be in slot 0, 3, 6.  Then we'll be
 *   in slot 1, 4, 7.  Then we'll be in 2, 5.  Then we'll be back to
 *   0, 3, and 6.  Since we could be in any frame we need to reserve
 *   for all of them.  Sucks, but that's what you gotta do.  Note that
 *   if we were instead scheduling 8 * 3 = 24 we'd do much better, but
 *   then we need more memory and time to do scheduling.
 * - If interval is 4, easy.
 * - If interval is 5, we again need interval 1.  The schedule will be
 *   0, 5, 2, 7, 4, 1, 6, 3, 0
 * - If interval is 6, we need interval 2.  0, 6, 4, 2.
 * - If interval is 7, we need interval 1.
 * - If interval is 8, we need interval 8.
 *
 * If you do the math, you'll see that we need to pretend that interval is
 * equal to the greatest_common_divisor(interval, periods_in_map).
 *
 * Note that at the moment this function tends to front-pack the schedule.
 * In some cases that's really non-ideal (it's hard to schedule things that
 * need to repeat every period).  In other cases it's perfect (you can easily
 * schedule bigger, less often repeating things).
 *
 * Here's the algorithm in action (8 periods, 5 bits per period):
 *  |**   |     |**   |     |**   |     |**   |     |   OK 2 bits, intv 2 at 0
 *  |*****|  ***|*****|  ***|*****|  ***|*****|  ***|   OK 3 bits, intv 3 at 2
 *  |*****|* ***|*****|  ***|*****|* ***|*****|  ***|   OK 1 bits, intv 4 at 5
 *  |**   |*    |**   |     |**   |*    |**   |     | Remv 3 bits, intv 3 at 2
 *  |***  |*    |***  |     |***  |*    |***  |     |   OK 1 bits, intv 6 at 2
 *  |**** |*  * |**** |   * |**** |*  * |**** |   * |   OK 1 bits, intv 1 at 3
 *  |**** |**** |**** | *** |**** |**** |**** | *** |   OK 2 bits, intv 2 at 6
 *  |*****|*****|*****| ****|*****|*****|*****| ****|   OK 1 bits, intv 1 at 4
 *  |*****|*****|*****| ****|*****|*****|*****| ****| FAIL 1 bits, intv 1
 *  |  ***|*****|  ***| ****|  ***|*****|  ***| ****| Remv 2 bits, intv 2 at 0
 *  |  ***| ****|  ***| ****|  ***| ****|  ***| ****| Remv 1 bits, intv 4 at 5
 *  |   **| ****|   **| ****|   **| ****|   **| ****| Remv 1 bits, intv 6 at 2
 *  |    *| ** *|    *| ** *|    *| ** *|    *| ** *| Remv 1 bits, intv 1 at 3
 *  |    *|    *|    *|    *|    *|    *|    *|    *| Remv 2 bits, intv 2 at 6
 *  |     |     |     |     |     |     |     |     | Remv 1 bits, intv 1 at 4
 *  |**   |     |**   |     |**   |     |**   |     |   OK 2 bits, intv 2 at 0
 *  |***  |     |**   |     |***  |     |**   |     |   OK 1 bits, intv 4 at 2
 *  |*****|     |** **|     |*****|     |** **|     |   OK 2 bits, intv 2 at 3
 *  |*****|*    |** **|     |*****|*    |** **|     |   OK 1 bits, intv 4 at 5
 *  |*****|***  |** **| **  |*****|***  |** **| **  |   OK 2 bits, intv 2 at 6
 *  |*****|*****|** **| ****|*****|*****|** **| ****|   OK 2 bits, intv 2 at 8
 *  |*****|*****|*****| ****|*****|*****|*****| ****|   OK 1 bits, intv 4 at 12
 *
 * This function is pretty generic and could be easily abstracted if anything
 * needed similar scheduling.
 *
 * Returns either -ENOSPC or a >= 0 start bit which should be passed to the
 * unschedule routine.  The map bitmap will be updated on a non-error result.
 */
static int pmap_schedule(unsigned long *map, int bits_per_period,
			 int periods_in_map, int num_bits,
			 int interval, int start, bool only_one_period)
{
	int interval_bits;
	int to_reserve;
	int first_end;
	int i;

	if (num_bits > bits_per_period)
		return -DWC_E_NO_SPACE;

	/* Adjust interval as per description */
	interval = gcd(interval, periods_in_map);

	interval_bits = bits_per_period * interval;
	to_reserve = periods_in_map / interval;

	/* If start has gotten us past interval then we can't schedule */
	if (start >= interval_bits)
		return -DWC_E_NO_SPACE;

	if (only_one_period)
		/* Must fit within same period as start; end at begin of next */
		first_end = (start / bits_per_period + 1) * bits_per_period;
	else
		/* Can fit anywhere in the first interval */
		first_end = interval_bits;

	/*
	 * We'll try to pick the first repetition, then see if that time
	 * is free for each of the subsequent repetitions.  If it's not
	 * we'll adjust the start time for the next search of the first
	 * repetition.
	 */
	while (start + num_bits <= first_end) {
		int end;

		/* Need to stay within this period */
		end = (start / bits_per_period + 1) * bits_per_period;

		/* Look for num_bits us in this microframe starting at start */
		start = bitmap_find_next_zero_area(map, end, start, num_bits,
						   0);

		/*
		 * We should get start >= end if we fail.  We might be
		 * able to check the next microframe depending on the
		 * interval, so continue on (start already updated).
		 */
		if (start >= end) {
			start = end;
			continue;
		}

		/* At this point we have a valid point for first one */
		for (i = 1; i < to_reserve; i++) {
			int ith_start = start + interval_bits * i;
			int ith_end = end + interval_bits * i;
			int ret;

			/* Use this as a dumb "check if bits are 0" */
			ret = bitmap_find_next_zero_area(
				map, ith_start + num_bits, ith_start, num_bits,
				0);

			/* We got the right place, continue checking */
			if (ret == ith_start)
				continue;

			/* Move start up for next time and exit for loop */
			ith_start = bitmap_find_next_zero_area(
				map, ith_end, ith_start, num_bits, 0);
			if (ith_start >= ith_end)
				/* Need a while new period next time */
				start = end;
			else
				start = ith_start - interval_bits * i;
			break;
		}

		/* If didn't exit the for loop with a break, we have success */
		if (i == to_reserve)
			break;
	}

	if (start + num_bits > first_end)
		return -DWC_E_NO_SPACE;

	for (i = 0; i < to_reserve; i++) {
		int ith_start = start + interval_bits * i;

		bitmap_set(map, ith_start, num_bits);
	}

	return start;
}

/**
 * pmap_unschedule() - Undo work done by pmap_schedule()
 *
 * @map:             See pmap_schedule().
 * @bits_per_period: See pmap_schedule().
 * @periods_in_map:  See pmap_schedule().
 * @num_bits:        The number of bits that was passed to schedule.
 * @interval:        The interval that was passed to schedule.
 * @start:           The return value from pmap_schedule().
 */
static void pmap_unschedule(unsigned long *map, int bits_per_period,
			    int periods_in_map, int num_bits,
			    int interval, int start)
{
	int interval_bits;
	int to_release;
	int i;

	/* Adjust interval as per description in pmap_schedule() */
	interval = gcd(interval, periods_in_map);

	interval_bits = bits_per_period * interval;
	to_release = periods_in_map / interval;

	for (i = 0; i < to_release; i++) {
		int ith_start = start + interval_bits * i;

		bitmap_clear(map, ith_start, num_bits);
	}
}

/*
 * cat_printf() - A printf() + strcat() helper
 *
 * This is useful for concatenating a bunch of strings where each string is
 * constructed using printf.
 *
 * @buf:   The destination buffer; will be updated to point after the printed
 *         data.
 * @size:  The number of bytes in the buffer (includes space for '\0').
 * @fmt:   The format for printf.
 * @...:   The args for printf.
 */
static void cat_printf(char **buf, size_t *size, const char *fmt, ...)
{
	va_list args;
	int i;

	if (*size == 0)
		return;

	va_start(args, fmt);
	i = vsnprintf(*buf, *size, fmt, args);
	va_end(args);

	if (i >= *size) {
		(*buf)[*size - 1] = '\0';
		*buf += *size;
		*size = 0;
	} else {
		*buf += i;
		*size -= i;
	}
}

/*
 * pmap_print() - Print the given periodic map
 *
 * Will attempt to print out the periodic schedule.
 *
 * @map:             See pmap_schedule().
 * @bits_per_period: See pmap_schedule().
 * @periods_in_map:  See pmap_schedule().
 * @period_name:     The name of 1 period, like "uFrame"
 * @units:           The name of the units, like "us".
 * @print_fn:        The function to call for printing.
 * @print_data:      Opaque data to pass to the print function.
 */
static void pmap_print(unsigned long *map, int bits_per_period,
		       int periods_in_map, const char *period_name,
		       const char *units,
		       void (*print_fn)(const char *str, void *data),
		       void *print_data)
{
	int period;

	for (period = 0; period < periods_in_map; period++) {
		char tmp[64];
		char *buf = tmp;
		size_t buf_size = sizeof(tmp);
		int period_start = period * bits_per_period;
		int period_end = period_start + bits_per_period;
		int start = 0;
		int count = 0;
		bool printed = false;
		int i;

		for (i = period_start; i < period_end + 1; i++) {
			/* Handle case when ith bit is set */
			if (i < period_end &&
			    bitmap_find_next_zero_area(map, i + 1,
						       i, 1, 0) != i) {
				if (count == 0)
					start = i - period_start;
				count++;
				continue;
			}

			/* ith bit isn't set; don't care if count == 0 */
			if (count == 0)
				continue;

			if (!printed)
				cat_printf(&buf, &buf_size, "%s %d: ",
					   period_name, period);
			else
				cat_printf(&buf, &buf_size, ", ");
			printed = true;

			cat_printf(&buf, &buf_size, "%d %s -%3d %s", start,
				   units, start + count - 1, units);
			count = 0;
		}

		if (printed)
			print_fn(tmp, print_data);
	}
}

static unsigned long *dwc_otg_hcd_get_ls_map(dwc_otg_hcd_t *hcd, dwc_otg_qh_t *qh)
{
	unsigned long *map;

	/* Don't expect to be missing a TT and be doing low speed scheduling */
	if (WARN_ON(!qh->dwc_tt))
		return NULL;

	/* Get the map and adjust if this is a multi_tt hub */
	map = qh->dwc_tt->periodic_bitmaps;
	if (qh->dwc_tt->usb_tt->multi)
		map += DWC_OTG_HCD_ELEMENTS_PER_LS_BITMAP * qh->ttport;

	return map;
}

struct dwc_otg_hcd_qh_print_data {
	dwc_otg_hcd_t *hcd;
	dwc_otg_qh_t *qh;
};

static void dwc_otg_hcd_qh_print(const char *str, void *data)
{
#if 0
	struct dwc_otg_hcd_qh_print_data *print_data = data;

	DWC_DEBUGPL(DBG_HCDV, "QH=%p ...%s\n", print_data->qh, str);
#endif
}

static void dwc_otg_hcd_qh_schedule_print(dwc_otg_hcd_t *hcd, dwc_otg_qh_t *qh)
{
	struct dwc_otg_hcd_qh_print_data print_data = { hcd, qh };
	int i;

	/*
	 * The printing functions are quite slow and inefficient.
	 * If we don't have tracing turned on, don't run unless the special
	 * define is turned on.
	 */
#ifndef DWC_OTG_HCD_PRINT_SCHEDULE
	return;
#endif

	if (qh->schedule_low_speed) {
		unsigned long *map = dwc_otg_hcd_get_ls_map(hcd, qh);

		DWC_DEBUGPL(DBG_HCDV, "QH=%p LS/FS trans: %d=>%d us @ %d us",
			    qh, qh->device_us,
			    DWC_OTG_HCD_ROUND_US_TO_SLICE(qh->device_us),
			    DWC_OTG_HCD_US_PER_SLICE * qh->ls_start_schedule_slice);

		if (map) {
			DWC_DEBUGPL(DBG_HCDV,
				    "QH=%p Whole low/full speed map %p now:\n",
				    qh, map);
			pmap_print(map, DWC_OTG_HCD_LS_PERIODIC_SLICES_PER_FRAME,
				   DWC_OTG_HCD_LS_SCHEDULE_FRAMES, "Frame ", "slices",
				   dwc_otg_hcd_qh_print, &print_data);
		}
	}

	for (i = 0; i < qh->num_hs_transfers; i++) {
#if 0
		struct dwc_otg_hcd_hs_transfer_time *trans_time = qh->hs_transfers + i;
		int uframe = trans_time->start_schedule_us /
			DWC_OTG_HCD_HS_PERIODIC_US_PER_UFRAME;
		int rel_us = trans_time->start_schedule_us %
			DWC_OTG_HCD_HS_PERIODIC_US_PER_UFRAME;

		DWC_DEBUGPL(DBG_HCDV,
			    "QH=%p HS trans #%d: %d us @ uFrame %d + %d us\n",
			    qh, i, trans_time->duration_us, uframe, rel_us);
#endif
	}
	if (qh->num_hs_transfers) {
		DWC_DEBUGPL(DBG_HCDV, "QH=%p Whole high speed map now:\n", qh);
		pmap_print(hcd->hs_periodic_bitmap,
			   DWC_OTG_HCD_HS_PERIODIC_US_PER_UFRAME,
			   DWC_OTG_HCD_HS_SCHEDULE_UFRAMES, "uFrame", "us",
			   dwc_otg_hcd_qh_print, &print_data);
	}
}

static int dwc_otg_hcd_ls_pmap_schedule(dwc_otg_hcd_t *hcd, dwc_otg_qh_t *qh,
					int search_slice)
{
	int slices = DIV_ROUND_UP(qh->device_us, DWC_OTG_HCD_US_PER_SLICE);
	unsigned long *map = dwc_otg_hcd_get_ls_map(hcd, qh);
	int slice;

	if (map == NULL)
		return -EINVAL;

	/*
	 * Schedule on the proper low speed map with our low speed scheduling
	 * parameters.  Note that we use the "device_interval" here since
	 * we want the low speed interval and the only way we'd be in this
	 * function is if the device is low speed.
	 *
	 * If we happen to be doing low speed and high speed scheduling for the
	 * same transaction (AKA we have a split) we always do low speed first.
	 * That means we can always pass "false" for only_one_period (that
	 * parameters is only useful when we're trying to get one schedule to
	 * match what we already planned in the other schedule).
	 */
	slice = pmap_schedule(map, DWC_OTG_HCD_LS_PERIODIC_SLICES_PER_FRAME,
			      DWC_OTG_HCD_LS_SCHEDULE_FRAMES, slices,
			      qh->device_interval, search_slice, false);

	if (slice < 0)
		return slice;

	qh->ls_start_schedule_slice = slice;
	return 0;
}

static void dwc_otg_hcd_ls_pmap_unschedule(dwc_otg_hcd_t *hcd, dwc_otg_qh_t *qh)
{
	int slices = DIV_ROUND_UP(qh->device_us, DWC_OTG_HCD_US_PER_SLICE);
	unsigned long *map = dwc_otg_hcd_get_ls_map(hcd, qh);

	/* Schedule should have failed, so no worries about no error code */
	if (map == NULL)
		return;

	pmap_unschedule(map, DWC_OTG_HCD_LS_PERIODIC_SLICES_PER_FRAME,
			DWC_OTG_HCD_LS_SCHEDULE_FRAMES, slices, qh->device_interval,
			qh->ls_start_schedule_slice);
}

static int dwc_otg_hcd_hs_pmap_schedule(dwc_otg_hcd_t *hcd, dwc_otg_qh_t *qh,
					bool only_one_period, int index)
{
	struct dwc_otg_hcd_hs_transfer_time *trans_time = qh->hs_transfers + index;
	int us;

	us = pmap_schedule(hcd->hs_periodic_bitmap,
			   DWC_OTG_HCD_HS_PERIODIC_US_PER_UFRAME,
			   DWC_OTG_HCD_HS_SCHEDULE_UFRAMES, trans_time->duration_us,
			   qh->host_interval, trans_time->start_schedule_us,
			   only_one_period);

	if (us < 0)
		return us;

	trans_time->start_schedule_us = us;
	return 0;
}

static void dwc_otg_hcd_hs_pmap_unschedule(dwc_otg_hcd_t *hcd,
					   dwc_otg_qh_t *qh, int index)
{
	struct dwc_otg_hcd_hs_transfer_time *trans_time = qh->hs_transfers + index;

	pmap_unschedule(hcd->hs_periodic_bitmap,
			DWC_OTG_HCD_HS_PERIODIC_US_PER_UFRAME,
			DWC_OTG_HCD_HS_SCHEDULE_UFRAMES, trans_time->duration_us,
			qh->host_interval, trans_time->start_schedule_us);
}

static int dwc_otg_hcd_uframe_schedule_split(dwc_otg_hcd_t *hcd, dwc_otg_qh_t *qh)
{
	int bytecount = dwc_hb_mult(qh->maxp) * dwc_max_packet(qh->maxp);
	int ls_search_slice;
	int err = 0;
	int host_interval_in_sched;

	/*
	 * The interval (how often to repeat) in the actual host schedule.
	 * See pmap_schedule() for gcd() explanation.
	 */
	host_interval_in_sched = gcd(qh->host_interval,
				     DWC_OTG_HCD_HS_SCHEDULE_UFRAMES);

	/*
	 * We always try to find space in the low speed schedule first, then
	 * try to find high speed time that matches.  If we don't, we'll bump
	 * up the place we start searching in the low speed schedule and try
	 * again.  To start we'll look right at the beginning of the low speed
	 * schedule.
	 *
	 * Note that this will tend to front-load the high speed schedule.
	 * We may eventually want to try to avoid this by either considering
	 * both schedules together or doing some sort of round robin.
	 */
	if (qh->ep_type == USB_ENDPOINT_XFER_ISOC && !qh->ep_is_in)
		ls_search_slice = 0;
	else
		ls_search_slice = 2 * DWC_OTG_HCD_SLICES_PER_UFRAME;

	while (ls_search_slice < DWC_OTG_HCD_LS_SCHEDULE_SLICES) {
		int start_s_uframe;
		int ssplit_s_uframe;
		int second_s_uframe;
		int rel_uframe;
		int first_count;
		int middle_count;
		int end_count;
		int first_data_bytes;
		int other_data_bytes;
		int i;

		if (qh->schedule_low_speed) {
			err = dwc_otg_hcd_ls_pmap_schedule(hcd, qh, ls_search_slice);

			/*
			 * If we got an error here there's no other magic we
			 * can do, so bail.  All the looping above is only
			 * helpful to redo things if we got a low speed slot
			 * and then couldn't find a matching high speed slot.
			 */
			if (err)
				return err;
		} else {
			/* Must be missing the tt structure?  Why? */
			WARN_ON_ONCE(1);
		}

		/*
		 * This will give us a number 0 - 7 if
		 * DWC_OTG_HCD_LS_SCHEDULE_FRAMES == 1, or 0 - 15 if == 2, or ...
		 */
		start_s_uframe = qh->ls_start_schedule_slice /
				 DWC_OTG_HCD_SLICES_PER_UFRAME;

		/* Get a number that's always 0 - 7 */
		rel_uframe = (start_s_uframe % 8);

		/*
		 * If we were going to start in uframe 7 then we would need to
		 * issue a start split in uframe 6, which spec says is not OK.
		 * Move on to the next full frame (assuming there is one).
		 *
		 * See 11.18.4 Host Split Transaction Scheduling Requirements
		 * bullet 1.
		 */
		if (rel_uframe == 7) {
			if (qh->schedule_low_speed)
				dwc_otg_hcd_ls_pmap_unschedule(hcd, qh);
			ls_search_slice =
				(qh->ls_start_schedule_slice /
				 DWC_OTG_HCD_LS_PERIODIC_SLICES_PER_FRAME + 1) *
				DWC_OTG_HCD_LS_PERIODIC_SLICES_PER_FRAME;
			continue;
		}

		/*
		 * For ISOC in:
		 * - start split            (frame -1)
		 * - complete split w/ data (frame +1)
		 * - complete split w/ data (frame +2)
		 * - ...
		 * - complete split w/ data (frame +num_data_packets)
		 * - complete split w/ data (frame +num_data_packets+1)
		 * - complete split w/ data (frame +num_data_packets+2, max 8)
		 *   ...though if frame was "0" then max is 7...
		 *
		 * For ISOC out we might need to do:
		 * - start split w/ data    (frame -1)
		 * - start split w/ data    (frame +0)
		 * - ...
		 * - start split w/ data    (frame +num_data_packets-2)
		 *
		 * For INTERRUPT in we might need to do:
		 * - start split            (frame -1)
		 * - complete split w/ data (frame +1)
		 * - complete split w/ data (frame +2)
		 * - complete split w/ data (frame +3, max 8)
		 *
		 * For INTERRUPT out we might need to do:
		 * - start split w/ data    (frame -1)
		 * - complete split         (frame +1)
		 * - complete split         (frame +2)
		 * - complete split         (frame +3, max 8)
		 *
		 * Start adjusting!
		 */
		ssplit_s_uframe = (start_s_uframe +
				   host_interval_in_sched - 1) %
				  host_interval_in_sched;
		if (qh->ep_type == USB_ENDPOINT_XFER_ISOC && !qh->ep_is_in)
			second_s_uframe = start_s_uframe;
		else
			second_s_uframe = start_s_uframe + 1;

		/* First data transfer might not be all 188 bytes. */
		first_data_bytes = 188 -
			DIV_ROUND_UP(188 * (qh->ls_start_schedule_slice %
					    DWC_OTG_HCD_SLICES_PER_UFRAME),
				     DWC_OTG_HCD_SLICES_PER_UFRAME);
		if (first_data_bytes > bytecount)
			first_data_bytes = bytecount;
		other_data_bytes = bytecount - first_data_bytes;

		/*
		 * For now, skip OUT xfers where first xfer is partial
		 *
		 * Main dwc2 code assumes:
		 * - INT transfers never get split in two.
		 * - ISOC transfers can always transfer 188 bytes the first
		 *   time.
		 *
		 * Until that code is fixed, try again if the first transfer
		 * couldn't transfer everything.
		 *
		 * This code can be removed if/when the rest of dwc2 handles
		 * the above cases.  Until it's fixed we just won't be able
		 * to schedule quite as tightly.
		 */
		if (!qh->ep_is_in &&
		    (first_data_bytes != min_t(int, 188, bytecount))) {
			DWC_DEBUGPL(DBG_HCDV,
				    "QH=%p avoiding broken 1st xfer (%d, %d)\n",
				    qh, first_data_bytes, bytecount);
			if (qh->schedule_low_speed)
				dwc_otg_hcd_ls_pmap_unschedule(hcd, qh);
			ls_search_slice = (start_s_uframe + 1) *
				DWC_OTG_HCD_SLICES_PER_UFRAME;
			continue;
		}

		/* Start by assuming transfers for the bytes */
		qh->num_hs_transfers = 1 + DIV_ROUND_UP(other_data_bytes, 188);

		/*
		 * Everything except ISOC OUT has extra transfers.  Rules are
		 * complicated.  See 11.18.4 Host Split Transaction Scheduling
		 * Requirements bullet 3.
		 */
		if (qh->ep_type == USB_ENDPOINT_XFER_INT) {
			if (rel_uframe == 6)
				qh->num_hs_transfers += 2;
			else
				qh->num_hs_transfers += 3;

			if (qh->ep_is_in) {
				/*
				 * First is start split, middle/end is data.
				 * Allocate full data bytes for all data.
				 */
				first_count = 4;
				middle_count = bytecount;
				end_count = bytecount;
			} else {
				/*
				 * First is data, middle/end is complete.
				 * First transfer and second can have data.
				 * Rest should just have complete split.
				 */
				first_count = first_data_bytes;
				middle_count = max_t(int, 4, other_data_bytes);
				end_count = 4;
			}
		} else {
			if (qh->ep_is_in) {
				int last;

				/* Account for the start split */
				qh->num_hs_transfers++;

				/* Calculate "L" value from spec */
				last = rel_uframe + qh->num_hs_transfers + 1;

				/* Start with basic case */
				if (last <= 6)
					qh->num_hs_transfers += 2;
				else
					qh->num_hs_transfers += 1;

				/* Adjust downwards */
				if (last >= 6 && rel_uframe == 0)
					qh->num_hs_transfers--;

				/* 1st = start; rest can contain data */
				first_count = 4;
				middle_count = min_t(int, 188, bytecount);
				end_count = middle_count;
			} else {
				/* All contain data, last might be smaller */
				first_count = first_data_bytes;
				middle_count = min_t(int, 188,
						     other_data_bytes);
				end_count = other_data_bytes % 188;
			}
		}

		/* Assign durations per uFrame */
		qh->hs_transfers[0].duration_us = HS_USECS_ISO(first_count);
		for (i = 1; i < qh->num_hs_transfers - 1; i++)
			qh->hs_transfers[i].duration_us =
				HS_USECS_ISO(middle_count);
		if (qh->num_hs_transfers > 1)
			qh->hs_transfers[qh->num_hs_transfers - 1].duration_us =
				HS_USECS_ISO(end_count);

		/*
		 * Assign start us.  The call below to dwc_otg_hcd_hs_pmap_schedule()
		 * will start with these numbers but may adjust within the same
		 * microframe.
		 */
		qh->hs_transfers[0].start_schedule_us =
			ssplit_s_uframe * DWC_OTG_HCD_HS_PERIODIC_US_PER_UFRAME;
		for (i = 1; i < qh->num_hs_transfers; i++)
			qh->hs_transfers[i].start_schedule_us =
				((second_s_uframe + i - 1) %
				 DWC_OTG_HCD_HS_SCHEDULE_UFRAMES) *
				DWC_OTG_HCD_HS_PERIODIC_US_PER_UFRAME;

		/* Try to schedule with filled in hs_transfers above */
		for (i = 0; i < qh->num_hs_transfers; i++) {
			err = dwc_otg_hcd_hs_pmap_schedule(hcd, qh, true, i);
			if (err)
				break;
		}

		/* If we scheduled all w/out breaking out then we're all good */
		if (i == qh->num_hs_transfers)
			break;

		for (; i >= 0; i--)
			dwc_otg_hcd_hs_pmap_unschedule(hcd, qh, i);

		if (qh->schedule_low_speed)
			dwc_otg_hcd_ls_pmap_unschedule(hcd, qh);

		/* Try again starting in the next microframe */
		ls_search_slice = (start_s_uframe + 1) * DWC_OTG_HCD_SLICES_PER_UFRAME;
	}

	if (ls_search_slice >= DWC_OTG_HCD_LS_SCHEDULE_SLICES)
		return -DWC_E_NO_SPACE;

	return 0;
}

static int dwc_otg_hcd_uframe_schedule_hs(dwc_otg_hcd_t *hcd, dwc_otg_qh_t *qh)
{
	/* In non-split host and device time are the same */
	WARN_ON(qh->host_us != qh->device_us);
	WARN_ON(qh->host_interval != qh->device_interval);
	WARN_ON(qh->num_hs_transfers != 1);

	/* We'll have one transfer; init start to 0 before calling scheduler */
	qh->hs_transfers[0].start_schedule_us = 0;
	qh->hs_transfers[0].duration_us = qh->host_us;

	return dwc_otg_hcd_hs_pmap_schedule(hcd, qh, false, 0);
}

static int dwc_otg_hcd_uframe_schedule_ls(dwc_otg_hcd_t *hcd, dwc_otg_qh_t *qh)
{
	/* In non-split host and device time are the same */
	WARN_ON(qh->host_us != qh->device_us);
	WARN_ON(qh->host_interval != qh->device_interval);
	WARN_ON(!qh->schedule_low_speed);

	/* Run on the main low speed schedule (no split = no hub = no TT) */
	return dwc_otg_hcd_ls_pmap_schedule(hcd, qh, 0);
}

static int dwc_otg_hcd_uframe_schedule(dwc_otg_hcd_t *hcd, dwc_otg_qh_t *qh)
{
	int ret;

	if (qh->dev_speed == USB_SPEED_HIGH)
		ret = dwc_otg_hcd_uframe_schedule_hs(hcd, qh);
	else if (!qh->do_split)
		ret = dwc_otg_hcd_uframe_schedule_ls(hcd, qh);
	else
		ret = dwc_otg_hcd_uframe_schedule_split(hcd, qh);

	if (ret)
		DWC_DEBUGPL(DBG_HCDV, "QH=%p Failed to schedule %d\n", qh, ret);
	else
		dwc_otg_hcd_qh_schedule_print(hcd, qh);

	return ret;
}

static void dwc_otg_hcd_uframe_unschedule(dwc_otg_hcd_t *hcd, dwc_otg_qh_t *qh)
{
	int i;

	for (i = 0; i < qh->num_hs_transfers; i++)
		dwc_otg_hcd_hs_pmap_unschedule(hcd, qh, i);

	if (qh->schedule_low_speed)
		dwc_otg_hcd_ls_pmap_unschedule(hcd, qh);

	DWC_DEBUGPL(DBG_HCDV, "QH=%p Unscheduled\n", qh);
}

static void dwc_otg_hcd_pick_first_frame(dwc_otg_hcd_t *hcd, dwc_otg_qh_t *qh)
{
	uint16_t frame_number;
	uint16_t earliest_frame;
	uint16_t next_active_frame;
	uint16_t relative_frame;
	uint16_t interval;

	/*
	 * Use the real frame number rather than the cached value as of the
	 * last SOF to give us a little extra slop.
	 */
	frame_number = dwc_otg_hcd_get_frame_number(hcd);

	/*
	 * We wouldn't want to start any earlier than the next frame just in
	 * case the frame number ticks as we're doing this calculation.
	 *
	 * NOTE: if we could quantify how long till we actually get scheduled
	 * we might be able to avoid the "+ 1" by looking at the upper part of
	 * HFNUM (the FRREM field).  For now we'll just use the + 1 though.
	 */
	earliest_frame = dwc_frame_num_inc(frame_number, 1);
	next_active_frame = earliest_frame;

	if (qh->dev_speed == USB_SPEED_HIGH || qh->do_split) {
		/*
		 * We're either at high speed or we're doing a split (which
		 * means we're talking high speed to a hub).  In any case
		 * the first frame should be based on when the first scheduled
		 * event is.
		 */
		WARN_ON(qh->num_hs_transfers < 1);

		relative_frame = qh->hs_transfers[0].start_schedule_us /
				 DWC_OTG_HCD_HS_PERIODIC_US_PER_UFRAME;

		/* Adjust interval as per high speed schedule */
		interval = gcd(qh->host_interval, DWC_OTG_HCD_HS_SCHEDULE_UFRAMES);
	} else {
		/*
		 * Low or full speed directly on dwc2.  Just about the same
		 * as high speed but on a different schedule and with slightly
		 * different adjustments.  Note that this works because when
		 * the host and device are both low speed then frames in the
		 * controller tick at low speed.
		 */
		relative_frame = qh->ls_start_schedule_slice /
				 DWC_OTG_HCD_LS_PERIODIC_SLICES_PER_FRAME;
		interval = gcd(qh->host_interval, DWC_OTG_HCD_LS_SCHEDULE_FRAMES);
	}

	/* Scheduler messed up if frame is past interval */
	WARN_ON(relative_frame >= interval);

	/*
	 * We know interval must divide (HFNUM_MAX_FRNUM + 1) now that we've
	 * done the gcd(), so it's safe to move to the beginning of the current
	 * interval like this.
	 *
	 * After this we might be before earliest_frame, but don't worry,
	 * we'll fix it...
	 */
	next_active_frame = (next_active_frame / interval) * interval;

	/*
	 * Actually choose to start at the frame number we've been
	 * scheduled for.
	 */
	next_active_frame = dwc_frame_num_inc(next_active_frame, relative_frame);

	/*
	 * We actually need 1 frame before since the next_active_frame is
	 * the frame number we'll be put on the ready list and we won't be on
	 * the bus until 1 frame later.
	 */
	next_active_frame = dwc_frame_num_dec(next_active_frame, 1);

	/*
	 * By now we might actually be before the earliest_frame.  Let's move
	 * up intervals until we're not.
	 */
	while (dwc_frame_num_gt(earliest_frame, next_active_frame))
		next_active_frame = dwc_frame_num_inc(next_active_frame,
						      interval);

	qh->next_active_frame = next_active_frame;
	qh->start_active_frame = next_active_frame;
}

static int dwc_otg_hcd_do_reserve(dwc_otg_hcd_t *hcd, dwc_otg_qh_t *qh)
{
	int status;

	status = dwc_otg_hcd_uframe_schedule(hcd, qh);

	if (status) {
		DWC_INFO("%s: Insufficient periodic bandwidth for " "periodic transfer.\n", __func__);
		return status;
	}

	/* Reserve the periodic channel. */
	hcd->periodic_channels++;

	/* Update claimed usecs per (micro)frame. */
	hcd->periodic_usecs += qh->host_us;

	dwc_otg_hcd_pick_first_frame(hcd, qh);

	return 0;
}

/**
 * Schedules an interrupt or isochronous transfer in the periodic schedule.
 *
 * @param hcd The HCD state structure for the DWC OTG controller.
 * @param qh QH for the periodic transfer. The QH should already contain the
 * scheduling information.
 *
 * @return 0 if successful, negative error code otherwise.
 */
static int schedule_periodic(dwc_otg_hcd_t *hcd, dwc_otg_qh_t *qh)
{
	int status = 0;

	status = periodic_channel_available(hcd);
	if (status) {
		DWC_INFO("%s: No host channel available for periodic " "transfer.\n", __func__);
		return status;
	}

	status = check_max_xfer_size(hcd, qh);
	if (status) {
		DWC_INFO("%s: Channel max transfer size too small " "for periodic transfer.\n", __func__);
		return status;
	}

	/* Cancel pending unreserve; if canceled OK, unreserve was pending */
	if (del_timer(&qh->unreserve_timer))
		WARN_ON(!qh->unreserve_pending);

	if (!qh->unreserve_pending) {
		status = dwc_otg_hcd_do_reserve(hcd, qh);
		if (status)
			return status;
	} else {
		if (dwc_frame_num_le(qh->next_active_frame,
				     hcd->frame_number))
			dwc_otg_hcd_pick_first_frame(hcd, qh);
	}

	qh->unreserve_pending = 0;

	if (hcd->core_if->dma_desc_enable) {
		/* Don't rely on SOF and start in ready schedule */
		DWC_LIST_INSERT_TAIL(&hcd->periodic_sched_ready,
				     &qh->qh_list_entry);
	} else {
		/* Always start in the inactive schedule. */
		DWC_LIST_INSERT_TAIL(&hcd->periodic_sched_inactive,
				     &qh->qh_list_entry);
	}

	return status;
}

/**
 * This function adds a QH to either the non periodic or periodic schedule if
 * it is not already in the schedule. If the QH is already in the schedule, no
 * action is taken.
 *
 * @return 0 if successful, negative error code otherwise.
 */
int dwc_otg_hcd_qh_add(dwc_otg_hcd_t *hcd, dwc_otg_qh_t *qh)
{
	int status = 0;
	gintmsk_data_t intr_mask = {.d32 = 0 };

	if (!DWC_LIST_EMPTY(&qh->qh_list_entry)) {
		/* QH already in a schedule. */
		return status;
	}

	/* Add the new QH to the appropriate schedule */
	if (dwc_qh_is_non_per(qh)) {
		/* Schedule right away */
		qh->start_active_frame = hcd->frame_number;
		qh->next_active_frame = qh->start_active_frame;
		/* Always start in the inactive schedule. */
		DWC_LIST_INSERT_TAIL(&hcd->non_periodic_sched_inactive,
				     &qh->qh_list_entry);
	} else {
		status = schedule_periodic(hcd, qh);
		if (status)
			return status;
		if (!hcd->periodic_qh_count) {
			intr_mask.b.sofintr = 1;
			DWC_MODIFY_REG32(&hcd->core_if->
					 core_global_regs->gintmsk,
					 intr_mask.d32, intr_mask.d32);
		}
		hcd->periodic_qh_count++;
	}

	return status;
}

/**
 * Removes an interrupt or isochronous transfer from the periodic schedule.
 *
 * @param hcd The HCD state structure for the DWC OTG controller.
 * @param qh QH for the periodic transfer.
 */
static void deschedule_periodic(dwc_otg_hcd_t *hcd, dwc_otg_qh_t *qh)
{
	bool did_modify;

	assert_spin_locked((spinlock_t *)(hcd->lock));

	did_modify = mod_timer(&qh->unreserve_timer,
			       jiffies + DWC_OTG_HCD_UNRESERVE_DELAY + 1);
	WARN_ON(did_modify);
	qh->unreserve_pending = 1;

	DWC_LIST_REMOVE_INIT(&qh->qh_list_entry);
}

/**
 * Removes a QH from either the non-periodic or periodic schedule.  Memory is
 * not freed.
 *
 * @param hcd The HCD state structure.
 * @param qh QH to remove from schedule. */
void dwc_otg_hcd_qh_remove(dwc_otg_hcd_t *hcd, dwc_otg_qh_t *qh)
{
	gintmsk_data_t intr_mask = {.d32 = 0 };

	if (DWC_LIST_EMPTY(&qh->qh_list_entry)) {
		/* QH is not in a schedule. */
		return;
	}

	if (dwc_qh_is_non_per(qh)) {
		if (hcd->non_periodic_qh_ptr == &qh->qh_list_entry) {
			hcd->non_periodic_qh_ptr =
			    hcd->non_periodic_qh_ptr->next;
		}
		DWC_LIST_REMOVE_INIT(&qh->qh_list_entry);
	} else {
		deschedule_periodic(hcd, qh);
		hcd->periodic_qh_count--;
		if (!hcd->periodic_qh_count) {
			intr_mask.b.sofintr = 1;
			DWC_MODIFY_REG32(&hcd->core_if->
					 core_global_regs->gintmsk,
					 intr_mask.d32, 0);
		}
	}
}

/*
 * Schedule the next continuing periodic split transfer
 */
static int dwc_otg_hcd_next_for_periodic_split(dwc_otg_hcd_t *hcd, dwc_otg_qh_t *qh,
					       u16 frame_number)
{
	uint16_t old_frame = qh->next_active_frame;
	u16 prev_frame_number = dwc_frame_num_dec(frame_number, 1);
	int missed = 0;
	uint16_t incr;

	/*
	 * Basically: increment 1 normally, but 2 right after the start split
	 * (except for ISOC out).
	 */
	if (old_frame == qh->start_active_frame &&
	    !(qh->ep_type == UE_ISOCHRONOUS && !qh->ep_is_in))
		incr = 2;
	else
		incr = 1;

	qh->next_active_frame = dwc_frame_num_inc(old_frame, incr);

	/*
	 * Note that it's OK for frame_number to be 1 frame past
	 * next_active_frame.  Remember that next_active_frame is supposed to
	 * be 1 frame _before_ when we want to be scheduled.  If we're 1 frame
	 * past it just means schedule ASAP.
	 *
	 * It's _not_ OK, however, if we're more than one frame past.
	 */
	if (dwc_frame_num_gt(prev_frame_number, qh->next_active_frame)) {
		/*
		 * OOPS, we missed.  That's actually pretty bad since
		 * the hub will be unhappy; try ASAP I guess.
		 */
		missed = dwc_frame_num_dec(prev_frame_number,
					   qh->next_active_frame);
		qh->next_active_frame = frame_number;
	}

	return missed;
}

static int dwc_otg_hcd_next_periodic_start(dwc_otg_hcd_t *hcd, dwc_otg_qh_t *qh,
					   u16 frame_number)
{
	int missed = 0;
	uint16_t interval = qh->host_interval;
	uint16_t prev_frame_number = dwc_frame_num_dec(frame_number, 1);

	qh->start_active_frame = dwc_frame_num_inc(qh->start_active_frame,
						   interval);

	/*
	 * The dwc_frame_num_gt() function used below won't work terribly well
	 * with if we just incremented by a really large intervals since the
	 * frame counter only goes to 0x3fff.  It's terribly unlikely that we
	 * will have missed in this case anyway.  Just go to exit.  If we want
	 * to try to do better we'll need to keep track of a bigger counter
	 * somewhere in the driver and handle overflows.
	 */
	if (interval >= 0x1000)
		goto exit;

	/*
	 * Test for misses, which is when it's too late to schedule.
	 *
	 * A few things to note:
	 * - We compare against prev_frame_number since start_active_frame
	 *   and next_active_frame are always 1 frame before we want things
	 *   to be active and we assume we can still get scheduled in the
	 *   current frame number.
	 * - It's possible for start_active_frame (now incremented) to be
	 *   next_active_frame if we got an EO MISS (even_odd miss) which
	 *   basically means that we detected there wasn't enough time for
	 *   the last packet and dwc2_hc_set_even_odd_frame() rescheduled us
	 *   at the last second.  We want to make sure we don't schedule
	 *   another transfer for the same frame.  My test webcam doesn't seem
	 *   terribly upset by missing a transfer but really doesn't like when
	 *   we do two transfers in the same frame.
	 * - Some misses are expected.  Specifically, in order to work
	 *   perfectly dwc2 really needs quite spectacular interrupt latency
	 *   requirements.  It needs to be able to handle its interrupts
	 *   completely within 125 us of them being asserted. That not only
	 *   means that the dwc2 interrupt handler needs to be fast but it
	 *   means that nothing else in the system has to block dwc2 for a long
	 *   time.  We can help with the dwc2 parts of this, but it's hard to
	 *   guarantee that a system will have interrupt latency < 125 us, so
	 *   we have to be robust to some misses.
	 */
	if (qh->start_active_frame == qh->next_active_frame ||
	    dwc_frame_num_gt(prev_frame_number, qh->start_active_frame)) {
		u16 ideal_start = qh->start_active_frame;
		int periods_in_map;

		/*
		 * Adjust interval as per gcd with map size.
		 * See pmap_schedule() for more details here.
		 */
		if (qh->do_split || qh->dev_speed == USB_SPEED_HIGH)
			periods_in_map = DWC_OTG_HCD_HS_SCHEDULE_UFRAMES;
		else
			periods_in_map = DWC_OTG_HCD_LS_SCHEDULE_FRAMES;
		interval = gcd(interval, periods_in_map);

		do {
			qh->start_active_frame = dwc_frame_num_inc(
				qh->start_active_frame, interval);
		} while (dwc_frame_num_gt(prev_frame_number,
					  qh->start_active_frame));

		missed = dwc_frame_num_dec(qh->start_active_frame,
					   ideal_start);
	}

exit:
	qh->next_active_frame = qh->start_active_frame;

	return missed;
}

/**
 * Deactivates a QH. For non-periodic QHs, removes the QH from the active
 * non-periodic schedule. The QH is added to the inactive non-periodic
 * schedule if any QTDs are still attached to the QH.
 *
 * For periodic QHs, the QH is removed from the periodic queued schedule. If
 * there are any QTDs still attached to the QH, the QH is added to either the
 * periodic inactive schedule or the periodic ready schedule and its next
 * scheduled frame is calculated. The QH is placed in the ready schedule if
 * the scheduled frame has been reached already. Otherwise it's placed in the
 * inactive schedule. If there are no QTDs attached to the QH, the QH is
 * completely removed from the periodic schedule.
 */
void dwc_otg_hcd_qh_deactivate(dwc_otg_hcd_t *hcd, dwc_otg_qh_t *qh,
			       int sched_next_periodic_split)
{
	uint16_t frame_number;
	int missed;

	if (dwc_qh_is_non_per(qh)) {
		dwc_otg_hcd_qh_remove(hcd, qh);
		if (!DWC_CIRCLEQ_EMPTY(&qh->qtd_list)) {
			/* Add back to inactive non-periodic schedule. */
			dwc_otg_hcd_qh_add(hcd, qh);
		}
		return;
	}

	frame_number = dwc_otg_hcd_get_frame_number(hcd);

	if (sched_next_periodic_split)
		missed = dwc_otg_hcd_next_for_periodic_split(hcd, qh, frame_number);
	else
		missed = dwc_otg_hcd_next_periodic_start(hcd, qh, frame_number);

	if (DWC_CIRCLEQ_EMPTY(&qh->qtd_list)) {
		dwc_otg_hcd_qh_remove(hcd, qh);
		return;
	}

	if (dwc_frame_num_le(qh->next_active_frame, frame_number))
		DWC_LIST_MOVE_TAIL(&hcd->periodic_sched_ready,
				   &qh->qh_list_entry);
	else
		DWC_LIST_MOVE_TAIL(&hcd->periodic_sched_inactive,
				   &qh->qh_list_entry);
}

/**
 * Initializes a QTD structure.
 *
 * @param qtd The QTD to initialize.
 * @param urb The URB to use for initialization.  */
void dwc_otg_hcd_qtd_init(dwc_otg_qtd_t *qtd, dwc_otg_hcd_urb_t *urb)
{
	dwc_memset(qtd, 0, sizeof(dwc_otg_qtd_t));
	qtd->urb = urb;
	if (dwc_otg_hcd_get_pipe_type(&urb->pipe_info) == UE_CONTROL) {
		/*
		 * The only time the QTD data toggle is used is on the data
		 * phase of control transfers. This phase always starts with
		 * DATA1.
		 */
		qtd->data_toggle = DWC_OTG_HC_PID_DATA1;
		qtd->control_phase = DWC_OTG_CONTROL_SETUP;
	}

	/* start split */
	qtd->complete_split = 0;
	qtd->isoc_split_pos = DWC_HCSPLIT_XACTPOS_ALL;
	qtd->isoc_split_offset = 0;
	qtd->in_process = 0;

	/* Store the qtd ptr in the urb to reference what QTD. */
	urb->qtd = qtd;
	return;
}

/**
 * This function adds a QTD to the QTD-list of a QH.  It will find the correct
 * QH to place the QTD into.  If it does not find a QH, then it will create a
 * new QH. If the QH to which the QTD is added is not currently scheduled, it
 * is placed into the proper schedule based on its EP type.
 *
 * @param[in] qtd The QTD to add
 * @param[in] hcd The DWC HCD structure
 * @param[out] qh out parameter to return queue head
 * @param atomic_alloc Flag to do atomic alloc if needed
 *
 * @return 0 if successful, negative error code otherwise.
 */
int dwc_otg_hcd_qtd_add(dwc_otg_qtd_t *qtd,
			dwc_otg_hcd_t *hcd, dwc_otg_qh_t **qh,
			int atomic_alloc)
{
	int retval = 0;

	dwc_otg_hcd_urb_t *urb = qtd->urb;

	/*
	 * Get the QH which holds the QTD-list to insert to. Create QH if it
	 * doesn't exist.
	 */
	if (*qh == NULL) {
		*qh = dwc_otg_hcd_qh_create(hcd, urb, atomic_alloc);
		if (*qh == NULL) {
			retval = -1;
			goto done;
		}
	}
	qtd->qh = *qh;
	retval = dwc_otg_hcd_qh_add(hcd, *qh);
	if (retval == 0) {
		DWC_CIRCLEQ_INSERT_TAIL(&((*qh)->qtd_list), qtd,
					qtd_list_entry);
	}

done:

	return retval;
}

#endif /* DWC_DEVICE_ONLY */
