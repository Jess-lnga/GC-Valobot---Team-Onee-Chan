// Code for team Onee~Chan - GC Valobot - Robopoly 2025 - 2026
// Author: Jérôme ESSOLA ELANGA - jerome.essolaelanga@epfl.ch
// Team members: Jérôme ESSOLA ELANGA


#include "ov7670_pio_dma.h"

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"

#include "ov7670.pio.h"   // généré par pico_generate_pio_header()

static PIO  s_pio        = pio0;
static uint s_sm         = 0;
static int  s_dma_chan   = -1;
static uint s_offset     = 0;

static uint s_data_base  = 0;
static uint s_pclk_pin   = 0;
static uint s_vsync_pin  = 0;
static uint s_href_pin   = 0;

static int  s_width      = 0;
static int  s_height     = 0;

static bool s_inited     = false;

bool ov7670_pio_dma_init(uint data_base_pin,
                         uint pclk_pin,
                         uint vsync_pin,
                         uint href_pin,
                         int width,
                         int height)
{
    if (s_inited) return true;
    if (width <= 0 || height <= 0) return false;

    s_data_base = data_base_pin;
    s_pclk_pin  = pclk_pin;
    s_vsync_pin = vsync_pin;
    s_href_pin  = href_pin;
    s_width     = width;
    s_height    = height;

    // 1) Les pins DATA + SYNC doivent être vues par PIO
    for (int i = 0; i < 8; i++) {
        gpio_set_function(s_data_base + i, GPIO_FUNC_PIO0);
    }
    gpio_set_function(s_pclk_pin,  GPIO_FUNC_PIO0);
    gpio_set_function(s_vsync_pin, GPIO_FUNC_PIO0);
    gpio_set_function(s_href_pin,  GPIO_FUNC_PIO0);

    // 2) Claim SM + DMA
    s_sm = pio_claim_unused_sm(s_pio, true);
    s_dma_chan = dma_claim_unused_channel(true);

    // 3) Charger le programme
    s_offset = pio_add_program(s_pio, &ov7670_capture_program);

    // 4) Config state machine
    pio_sm_config c = ov7670_capture_program_get_default_config(s_offset);

    // in pins base = D0
    sm_config_set_in_pins(&c, s_data_base);

    // Shift IN : shift_left (false), autopush=16 -> 1 pixel = 16 bits
    sm_config_set_in_shift(&c, false, true, 16);

    // IMPORTANT : ne pas JOIN RX ici, car le programme utilise pull block (TX FIFO)
    // sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX); // <-- NE PAS METTRE

    // clkdiv=1 : le timing est dicté par WAIT sur GPIO (PCLK)
    sm_config_set_clkdiv(&c, 1.0f);

    // D0..D7 en entrée
    pio_sm_set_consecutive_pindirs(s_pio, s_sm, s_data_base, 8, false);

    // Init SM (désactivée)
    pio_sm_init(s_pio, s_sm, s_offset, &c);
    pio_sm_set_enabled(s_pio, s_sm, false);

    s_inited = true;
    return true;
}

bool ov7670_pio_dma_capture(uint16_t *buf)
{
    if (!s_inited || !buf) return false;

    const uint32_t count = (uint32_t)s_width * (uint32_t)s_height;

    // Stop + reset SM
    pio_sm_set_enabled(s_pio, s_sm, false);
    pio_sm_clear_fifos(s_pio, s_sm);
    pio_sm_restart(s_pio, s_sm);

    // Le PIO attend : HEIGHT-1 puis WIDTH-1
    pio_sm_put_blocking(s_pio, s_sm, (uint32_t)(s_height - 1));
    pio_sm_put_blocking(s_pio, s_sm, (uint32_t)(s_width  - 1));

    // Config DMA : lire depuis RX FIFO PIO (16 bits) -> buf[]
    dma_channel_config cfg = dma_channel_get_default_config(s_dma_chan);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);
    channel_config_set_dreq(&cfg, pio_get_dreq(s_pio, s_sm, false)); // RX DREQ

    dma_channel_configure(
        s_dma_chan,
        &cfg,
        buf,                 // write addr
        &s_pio->rxf[s_sm],   // read addr
        count,               // transfers (uint16_t)
        true                 // start
    );

    // Démarrer la SM : elle attend VSYNC puis capture
    pio_sm_set_enabled(s_pio, s_sm, true);

    // Timeout "large" (évite noir si blanking long ou fps bas)
    absolute_time_t t0 = get_absolute_time();
    while (dma_channel_is_busy(s_dma_chan)) {
        if (absolute_time_diff_us(t0, get_absolute_time()) > 500000) { // 500 ms
            dma_channel_abort(s_dma_chan);
            pio_sm_set_enabled(s_pio, s_sm, false);
            return false;
        }
        tight_loop_contents();
    }

    // Stop SM (sinon elle repartirait sur VSYNC suivant)
    pio_sm_set_enabled(s_pio, s_sm, false);
    return true;
}
