/*
Copyright 2011,2013 Jun Wako <wakojun@gmail.com>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdbool.h>
#include<avr/io.h>
#include<util/delay.h>
#include "ps2.h"
#include "ps2_mouse.h"
#include "report.h"
#include "host.h"
#include "timer.h"
#include "print.h"
#include "debug.h"

#define MOUSE_SET_RESOLUTION   0xE8
#define MOUSE_STATUS_REQUEST   0xE9
#define MOUSE_SET_STREAM_MODE  0xEA
#define MOUSE_SET_REMOTE_MODE  0xF0
#define MOUSE_SET_SAMPLE_RATE  0xF3
#define MOUSE_ENABLE     0xF4
#define MOUSE_DISABLE    0xF5
#define MOUSE_RESET      0xFF
#define MOUSE_RESEND     0xFE
#define MOUSE_ERROR      0xFC

#define SYNAP_SET_MODE    0x14
#define SYNAP_TUNNEL_CMD  0x28

#define SYNAP_MODE_ABSOLUTE  0x80
#define SYNAP_MODE_HIGH_RATE  0x40
#define SYNAP_MODE_TRANSPARENT 0x20
#define SYNAP_MODE_EXT_W 0x04
#define SYNAP_MODE_W  0x01

static report_mouse_t mouse_report = {};
static void print_usb_data(void);

uint8_t mouse_send(uint8_t data) {
  uint8_t rcv = ps2_host_send(data);
  if (rcv != PS2_ACK) xprintf("Send %02x failed! : %02x", data, rcv);
  return rcv;
}

uint8_t synap_encode_byte(uint8_t data) {
  uint8_t rcv;
  mouse_send(MOUSE_SET_RESOLUTION);
  mouse_send(data >> 6 & 0x03);
  mouse_send(MOUSE_SET_RESOLUTION);
  mouse_send(data >> 4 & 0x03);
  mouse_send(MOUSE_SET_RESOLUTION);
  mouse_send(data >> 2 & 0x03);
  mouse_send(MOUSE_SET_RESOLUTION);
  rcv = mouse_send(data & 0x03);
  return rcv;
}

uint8_t synap_encoded_command(uint8_t data, uint8_t cmd) {
    print("sending encoded command "); phex(cmd); print("with data "); phex(data); print("\n");
    uint8_t rcv = mouse_send(MOUSE_DISABLE);
    rcv = synap_encode_byte(data);
    rcv = mouse_send(MOUSE_SET_SAMPLE_RATE);
    rcv = mouse_send(cmd);
    return rcv;
}

uint8_t synap_set_mode(uint8_t mode) {
  return synap_encoded_command(mode, SYNAP_SET_MODE);
}

uint8_t synap_tunnel_cmd(uint8_t cmd) {
  return synap_encoded_command(cmd, SYNAP_TUNNEL_CMD);
}

/* supports only 3 button mouse at this time */
uint8_t ps2_mouse_init(void) {
    uint8_t rcv;

    ps2_host_init();

    _delay_ms(1000);    // wait for powering up

    ps2_host_recv();

    // send Reset
    rcv = ps2_host_send(0xFF);
    print("ps2_mouse_init: send Reset: ");
    phex(rcv); print("\n");

    // read completion code of BAT
    rcv = ps2_host_recv_response();
    while (rcv != 0xAA && rcv != 0xFC) {
    print("ps2_mouse_init: waiting for BAT response \n");
        rcv = ps2_host_recv_response();
   }

    // read Device ID
    rcv = ps2_host_recv_response();
    print("ps2_mouse_init: read DevID: "); phex(rcv); print("\n");
    //phex(rcv); phex(ps2_error); print("\n");

    println("setting touchpad mode");
    synap_set_mode(SYNAP_MODE_ABSOLUTE | SYNAP_MODE_W | SYNAP_MODE_EXT_W);

    println("sending enable");
    mouse_send(MOUSE_ENABLE);

    return 0;
}

#define X_IS_NEG  (mouse_report.buttons & (1<<PS2_MOUSE_X_SIGN))
#define Y_IS_NEG  (mouse_report.buttons & (1<<PS2_MOUSE_Y_SIGN))
#define X_IS_OVF  (mouse_report.buttons & (1<<PS2_MOUSE_X_OVFLW))
#define Y_IS_OVF  (mouse_report.buttons & (1<<PS2_MOUSE_Y_OVFLW))
void ps2_mouse_task(void)
{
    enum { SCROLL_NONE, SCROLL_BTN, SCROLL_SENT };
    static uint8_t scroll_state = SCROLL_NONE;
    static uint8_t buttons_prev = 0;

    static uint8_t buffer[6];
    static int buffer_cur = 0;

    /* receives packet from mouse */
    uint8_t rcv;
    rcv = ps2_host_recv();
    if (ps2_error == PS2_ERR_NODATA) {
      return;
    }

    // discard bytes until we find one with an absolute mode pattern
    if (buffer_cur == 0 && rcv != 0x84) {
      return;
    }

    print("ps2_mouse: byte: "); phex(rcv); print("\n");

    if (buffer_cur == 3) {
/*      uint8_t w0 = (rcv & 0x4) >> 2;
      // check if we're on track for an encapsulation packet
      uint8_t w1 = (buffer[0] & 0x4) >> 1;
      uint8_t w23 = (buffer[0] & 0x30) >> 2;*/
      if (rcv != 0xC4) {
        buffer_cur = 0;
        println("not an encapsulation packet");
        return;
      } else {
        println("looks like encapsulation packet");
      }
    }


    buffer[buffer_cur++] = rcv;

    if (buffer_cur < 6) {
      phex(buffer_cur); println(" waiting for full packet");
      return;
    }

    buffer_cur = 0;

    mouse_report.buttons = buffer[1];
    mouse_report.x = buffer[4];
    mouse_report.y = buffer[5];

    /* if mouse moves or buttons state changes */
    if (mouse_report.x || mouse_report.y ||
            ((mouse_report.buttons ^ buttons_prev) & PS2_MOUSE_BTN_MASK)) {

#ifdef PS2_MOUSE_DEBUG
        print("ps2_mouse raw: [");
        phex(mouse_report.buttons); print("|");
        print_hex8((uint8_t)mouse_report.x); print(" ");
        print_hex8((uint8_t)mouse_report.y); print("]\n");
#endif

        buttons_prev = mouse_report.buttons;

        // PS/2 mouse data is '9-bit integer'(-256 to 255) which is comprised of sign-bit and 8-bit value.
        // bit: 8    7 ... 0
        //      sign \8-bit/
        //
        // Meanwhile USB HID mouse indicates 8bit data(-127 to 127), note that -128 is not used.
        //
        // This converts PS/2 data into HID value. Use only -127-127 out of PS/2 9-bit.
        mouse_report.x = X_IS_NEG ?
                          ((!X_IS_OVF && -127 <= mouse_report.x && mouse_report.x <= -1) ?  mouse_report.x : -127) :
                          ((!X_IS_OVF && 0 <= mouse_report.x && mouse_report.x <= 127) ? mouse_report.x : 127);
        mouse_report.y = Y_IS_NEG ?
                          ((!Y_IS_OVF && -127 <= mouse_report.y && mouse_report.y <= -1) ?  mouse_report.y : -127) :
                          ((!Y_IS_OVF && 0 <= mouse_report.y && mouse_report.y <= 127) ? mouse_report.y : 127);

        // remove sign and overflow flags
        mouse_report.buttons &= PS2_MOUSE_BTN_MASK;

        // invert coordinate of y to conform to USB HID mouse
        mouse_report.y = -mouse_report.y;


#if PS2_MOUSE_SCROLL_BTN_MASK
        static uint16_t scroll_button_time = 0;
        if ((mouse_report.buttons & (PS2_MOUSE_SCROLL_BTN_MASK)) == (PS2_MOUSE_SCROLL_BTN_MASK)) {
            if (scroll_state == SCROLL_NONE) {
                scroll_button_time = timer_read();
                scroll_state = SCROLL_BTN;
            }

            // doesn't send Scroll Button
            //mouse_report.buttons &= ~(PS2_MOUSE_SCROLL_BTN_MASK);

            if (mouse_report.x || mouse_report.y) {
                scroll_state = SCROLL_SENT;

                mouse_report.v = -mouse_report.y/(PS2_MOUSE_SCROLL_DIVISOR_V);
                mouse_report.h =  mouse_report.x/(PS2_MOUSE_SCROLL_DIVISOR_H);
                mouse_report.x = 0;
                mouse_report.y = 0;
                //host_mouse_send(&mouse_report);
            }
        }
        else if ((mouse_report.buttons & (PS2_MOUSE_SCROLL_BTN_MASK)) == 0) {
#if PS2_MOUSE_SCROLL_BTN_SEND
            if (scroll_state == SCROLL_BTN &&
                    TIMER_DIFF_16(timer_read(), scroll_button_time) < PS2_MOUSE_SCROLL_BTN_SEND) {
                // send Scroll Button(down and up at once) when not scrolled
                mouse_report.buttons |= (PS2_MOUSE_SCROLL_BTN_MASK);
                host_mouse_send(&mouse_report);
                _delay_ms(100);
                mouse_report.buttons &= ~(PS2_MOUSE_SCROLL_BTN_MASK);
            }
#endif
            scroll_state = SCROLL_NONE;
        }
        // doesn't send Scroll Button
        mouse_report.buttons &= ~(PS2_MOUSE_SCROLL_BTN_MASK);
#endif


        host_mouse_send(&mouse_report);
        print_usb_data();
    }
    // clear report
    mouse_report.x = 0;
    mouse_report.y = 0;
    mouse_report.v = 0;
    mouse_report.h = 0;
    mouse_report.buttons = 0;
}

static void print_usb_data(void)
{
    if (!debug_mouse) return;
    print("ps2_mouse usb: [");
    phex(mouse_report.buttons); print("|");
    print_hex8((uint8_t)mouse_report.x); print(" ");
    print_hex8((uint8_t)mouse_report.y); print(" ");
    print_hex8((uint8_t)mouse_report.v); print(" ");
    print_hex8((uint8_t)mouse_report.h); print("]\n");
}


/* PS/2 Mouse Synopsis
 * http://www.computer-engineering.org/ps2mouse/
 *
 * Command:
 * 0xFF: Reset
 * 0xF6: Set Defaults Sampling; rate=100, resolution=4cnt/mm, scaling=1:1, reporting=disabled
 * 0xF5: Disable Data Reporting
 * 0xF4: Enable Data Reporting
 * 0xF3: Set Sample Rate
 * 0xF2: Get Device ID
 * 0xF0: Set Remote Mode
 * 0xEB: Read Data
 * 0xEA: Set Stream Mode
 * 0xE9: Status Request
 * 0xE8: Set Resolution
 * 0xE7: Set Scaling 2:1
 * 0xE6: Set Scaling 1:1
 *
 * Mode:
 * Stream Mode: devices sends the data when it changs its state
 * Remote Mode: host polls the data periodically
 *
 * This code uses Remote Mode and polls the data with Read Data(0xEB).
 *
 * Data format:
 * byte|7       6       5       4       3       2       1       0
 * ----+--------------------------------------------------------------
 *    0|Yovflw  Xovflw  Ysign   Xsign   1       Middle  Right   Left
 *    1|                    X movement
 *    2|                    Y movement
 */
