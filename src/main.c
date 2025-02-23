#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/timer.h"
#include "hardware/sync.h"
#include "ssd1306/ssd1306.h"
#include "neopixel.c"
#include "bitmap.h"

#define MICROPHONE_PIN          28
#define MICROPHONE_CHANNEL      2
#define NPLED_PIN               7
#define NPLED_COUNT             25
#define I2C_SDA_PIN             14
#define I2C_SCL_PIN             15
#define BTN_A_PIN               5
#define DEBOUNCE_DELAY_MS       50

// Parâmetros e macros do ADC.
#define ADC_CLK_REF             48000000
#define ADC_CLOCK_DIV           12000.f //48M/(12000) = 4kSps
#define SAMPLES                 4096   // Número de amostras que serão feitas do ADC.
#define ADC_ADJUST(x)           (x * 3.3f / (1 << 12u) - 1.65f) // Ajuste do valor do ADC para Volts.

// Frequências para as notas da guitarra (em Hz)
#define FREQ_E2                 82.41
#define FREQ_A2                 110.00
#define FREQ_D3                 146.83
#define FREQ_G3                 196.00
#define FREQ_B3                 246.94
#define FREQ_E4                 329.63

#define FREQ_RESOLUTION         (float)((ADC_CLK_REF/ADC_CLOCK_DIV)/SAMPLES)
#define PI                      3.14159265358979323846
#define NUM_OF_STRINGS          6
#define DELTA_FREQ_FOR_TUNNING  0.1225 // Desvio relativo entre um tom, e os tons abaixo ou acima
#define MAX_DEV_TO_BE_TUNED     0.5 // Margem para considerar afinado: 0,5 Hz abaixo ou acima
#define REST_TIME               20000


// Enum para os estados da máquina
typedef enum {
    WELCOME,
    IDLE,
    DETECTING,
    TUNING,
    LOW,
    HIGH,
    CORRECT
} state;

// Enum notas da guitarra em afinação padrão
enum note {E2, A2, D3, G3, B3, E4, UNKNOWN};
 
//Enum para condição de afinação da corda
enum string_tune {VERY_DOWN, DOWN, TUNED, UP, VERY_UP};

//Estrutura para representar uma corda da guitara
typedef struct {
    int note;
    float desired_freq;
} string_t;

// Estrutura para representar números complexos
typedef struct {
    float r;
    float i;
} complex_t;

// Preparar área de renderização para o display
struct render_area frame_area = {
    start_column : 0,
    end_column : ssd1306_width - 1,
    start_page : 0,
    end_page : ssd1306_n_pages - 1
};

// Canal e configurações do DMA
uint dma_channel;
dma_channel_config dma_cfg;

// Buffer de amostras do ADC.
uint16_t adc_buffer[SAMPLES];
complex_t fft_data[SAMPLES];
void adc_sample_mic();
void dsp_fft_pre_calc(uint16_t*, int, complex_t*);
void dsp_hann_window(complex_t*, int);
void dsp_fft(complex_t*, int);
float dsp_magnitude(complex_t);
float dsp_find_peak(complex_t*, int, float);
bool check_note(float);


uint8_t oled[ssd1306_buffer_length];
void print_on_oled(uint8_t *, char[][16], int, int, int);

void oled_config(){
     // Inicialização do i2c
    i2c_init(i2c1, ssd1306_i2c_clock * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // Processo de inicialização completo do OLED SSD1306
    ssd1306_init();
    calculate_render_area_buffer_length(&frame_area);
    memset(oled, 0, ssd1306_buffer_length);
    render_on_display(oled, &frame_area);
}

void adc_dma_config(){
    adc_gpio_init(MICROPHONE_PIN);
    adc_init();
    adc_select_input(MICROPHONE_CHANNEL);

    adc_fifo_setup(
        true,   // Habilitar FIFO
        true,   // Habilitar request de dados do DMA
        1,      // Threshold para ativar request DMA é 1 leitura do ADC
        false,  // Não usar bit de erro
        false   // Não fazer downscale das amostras para 8-bits, manter 12-bits.
    );

    adc_set_clkdiv(ADC_CLOCK_DIV);

    dma_channel = dma_claim_unused_channel(true);

    // Configurações do DMA.
    dma_cfg = dma_channel_get_default_config(dma_channel);

    channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_16); // Tamanho da transferência é 16-bits (usamos uint16_t para armazenar valores do ADC)
    channel_config_set_read_increment(&dma_cfg, false);            // Desabilita incremento do ponteiro de leitura (lemos de um único registrador)
    channel_config_set_write_increment(&dma_cfg, true);            // Habilita incremento do ponteiro de escrita (escrevemos em um array/buffer)

    channel_config_set_dreq(&dma_cfg, DREQ_ADC);                   // Usamos a requisição de dados do ADC

}

void set_LED_matrix(int);
volatile bool button_pressed = false;
volatile bool timer_expired = false;
volatile state current_state = WELCOME;
bool timer_started = false;
alarm_id_t detecting_alarm = 0;
static void gpio_callback(uint gpio, uint32_t events);
static int64_t timer_callback(alarm_id_t id, void *user_data);
char *note_labels[] = {"E2", "A2", "D3", "G3", "B3", "E4"};

string_t strings[NUM_OF_STRINGS] = {
    {E2, FREQ_E2}, // E2
    {A2, FREQ_A2}, // A2
    {D3, FREQ_D3}, // D3
    {G3, FREQ_G3}, // G3
    {B3, FREQ_B3}, // B3
    {E4, FREQ_E4}  // E4
};

string_t current_string = {UNKNOWN, 0.0f};

char message[8][16] = {0};
float detected_freq = 0.0;
float dev_freq; 

int main() {
    
    int led_index;
    bool up_down = true;
    stdio_init_all();
    
    /*
    //Configuração para uso de bitmap
        ssd1306_t ssd_bm;
        ssd1306_init_bm(&ssd_bm, BITMAP_WIDTH, BITMAP_HEIGHT, false, 0x3C, i2c1);
        ssd1306_config(&ssd_bm);
    */ 

    adc_dma_config();
    
    npInit(NPLED_PIN, NPLED_COUNT);
    for(int i = 0; i < NPLED_COUNT; i++){
        npSetLED(i, 0, 0, 0);
    }
    npWrite();

    gpio_init(BTN_A_PIN);
    gpio_set_dir(BTN_A_PIN, GPIO_IN);
    gpio_pull_up(BTN_A_PIN);

    gpio_set_irq_enabled_with_callback(BTN_A_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
   
    /*
    memcpy(oled, image_bitmap, sizeof(image_bitmap));
    render_on_display(oled, &frame_area);
    sleep_ms(3000);  // exibe o bitmap por 3 segundos
    */
    
    /*
    ssd1306_draw_bitmap(&ssd_bm, image_bitmap);
    sleep_ms(3000);
    
    */
    oled_config();

    memset(oled, 0, ssd1306_buffer_length);
    render_on_display(oled, &frame_area);

    sleep_ms(100);

    while (1) {
        if (up_down) {
            led_index= (led_index + 1) % 25;
            up_down = led_index == 24 ? false : true;
        } else {
            led_index = (led_index - 1) % 25;
            up_down = led_index == 0 ? true : false;
        }
        npClear();
        printf("%d\n", current_state);
        switch (current_state) {
            case WELCOME:
                memset(message, 0, sizeof(message));
                    strncpy(message[0], "  Embarcatech  ", 16);
                    strncpy(message[1], "               ", 16);
                    strncpy(message[2], "    Afinador   ", 16);
                    strncpy(message[3], "    Digital    ", 16);
                   
                print_on_oled(oled, message, 4, 5, 0);
                sleep_ms(2000);
                current_state = DETECTING;
                led_index = - 1;
                break;

            case DETECTING:
                if(!timer_started){
                    detecting_alarm = add_alarm_in_ms(REST_TIME, timer_callback, NULL, false);
                    timer_started = true;
                }
                memset(message, 0, sizeof(message));
                strncpy(message[0], "   Captando    ", 16);
                strncpy(message[1], "   o som da    ", 16);
                strncpy(message[2], "   guitarra    ", 16);
                                            
                print_on_oled(oled, message, 3, 6, 16);
                
                npSetLED(led_index,0, 0, 128);
                npWrite();
                
                adc_sample_mic();
                dsp_fft_pre_calc(adc_buffer, SAMPLES, fft_data);
                dsp_hann_window(fft_data, SAMPLES);
                dsp_fft(fft_data, SAMPLES);
                detected_freq = dsp_find_peak(fft_data, SAMPLES, FREQ_RESOLUTION);

                if(timer_expired){
                    current_state = IDLE;
                    timer_expired = false;
                    break;
                }

                if(check_note(detected_freq)) {
                   cancel_alarm(detecting_alarm);
                   timer_started = false;
                   current_state = TUNING;
                } else {
                    current_state = DETECTING;
                }
                break;

            case TUNING:
                dev_freq = detected_freq - current_string.desired_freq;
                if (fabs(dev_freq) < MAX_DEV_TO_BE_TUNED){
                    current_state = CORRECT;
                    set_LED_matrix(TUNED);
                }else if(dev_freq > 0.5){
                    current_state = HIGH;
                    if (dev_freq/current_string.desired_freq > DELTA_FREQ_FOR_TUNNING/2)
                        set_LED_matrix(VERY_UP);
                    else
                        set_LED_matrix(UP);
                }else if(dev_freq < -0.5){
                    current_state = LOW;
                    if (dev_freq/current_string.desired_freq < -DELTA_FREQ_FOR_TUNNING/2)
                        set_LED_matrix(VERY_DOWN);
                    else
                        set_LED_matrix(DOWN);
                }
                break;
            case LOW:
                memset(message, 0, sizeof(message));
                strncpy(message[0], "    Nota: ", 12);
                strncat(message[0] ,note_labels[current_string.note], 2);
                strncpy(message[1], "              ", 16);
                strncpy(message[2], "   Aperte a  ", 16);
                strncpy(message[3], "     corda    ", 16);
                print_on_oled(oled, message, 4, 6, 8);

                adc_sample_mic();
                dsp_fft_pre_calc(adc_buffer, SAMPLES, fft_data);
                dsp_hann_window(fft_data, SAMPLES);
                dsp_fft(fft_data, SAMPLES);
                detected_freq = dsp_find_peak(fft_data, SAMPLES, FREQ_RESOLUTION);
                current_state = check_note(detected_freq) ? TUNING : DETECTING;
                if (current_state == DETECTING){
                    led_index = -1;
                    up_down = true;
                }
                   
                break;

            case HIGH:
                memset(message, 0, sizeof(message));
                strncpy(message[0], "    Nota: ", 12);
                strncat(message[0] ,note_labels[current_string.note], 2);
                strncpy(message[1], "               ", 16);
                strncpy(message[2], "   Afrouxe a  ", 16);
                strncpy(message[3], "     corda    ", 16);
                print_on_oled(oled, message, 4, 6, 8);

                adc_sample_mic();
                dsp_fft_pre_calc(adc_buffer, SAMPLES, fft_data);
                dsp_hann_window(fft_data, SAMPLES);
                dsp_fft(fft_data, SAMPLES);
                detected_freq = dsp_find_peak(fft_data, SAMPLES, FREQ_RESOLUTION);
                current_state = check_note(detected_freq) ? TUNING : DETECTING;
                if (current_state == DETECTING){
                    led_index = -1;
                    up_down = true;
                }
                break;

            case CORRECT:
                memset(message, 0, sizeof(message));
                strncpy(message[0], "   Nota: ", 12);
                strncat(message[0] ,note_labels[current_string.note], 2);
                strncpy(message[1], "              ", 16);
                strncpy(message[2], "    Corda     ", 16);
                strncpy(message[3], "   Afinada!   ", 16);
                print_on_oled(oled, message, 4, 6, 8);
                current_state = DETECTING;  // Espera por outro sinal de áudio
                led_index = - 1;
                up_down = true;
                timer_started = false;
                sleep_ms(2000);
                break;
            case IDLE:
                memset(message, 0, sizeof(message));
                strncpy(message[0], "  Embarcatech  ", 16);
                strncpy(message[1], "               ", 16);
                strncpy(message[2], "    Afinador   ", 16);
                strncpy(message[3], "    Digital    ", 16); 
                print_on_oled(oled, message, 4, 5, 0);
                sleep_ms(1000);
                current_state = IDLE;
                break;  
        }
       
        if(button_pressed && current_state == IDLE){
            current_state = DETECTING;
            led_index = -1;
            up_down = true;
            button_pressed = false;
            timer_started = false;
        }
        sleep_ms(100);       
    }

    return 0;
}

void print_on_oled(uint8_t* oled, char message[][16], int num_of_words, int x, int y) {
    memset(oled, 0, ssd1306_buffer_length);
    render_on_display(oled, &frame_area);

    for (uint i = 0; i < num_of_words; i++) {
        ssd1306_draw_string(oled, x, y, message[i]);
        y += 8;
    }
    render_on_display(oled, &frame_area);
}

void adc_sample_mic() {
    adc_fifo_drain(); // Limpa o FIFO do ADC.
    adc_run(false);   // Desliga o ADC (se estiver ligado) para configurar o DMA.

    dma_channel_configure(
        dma_channel,
        &dma_cfg,
        adc_buffer,         // Escreve no buffer.
        &(adc_hw->fifo),    // Lê do ADC.
        SAMPLES,            // Faz SAMPLES amostras.
        true                // Liga o DMA.
    );

    // Liga o ADC e espera acabar a leitura.
    adc_run(true);
    dma_channel_wait_for_finish_blocking(dma_channel);

    // Acabou a leitura, desliga o ADC de novo.
    adc_run(false);
}

void dsp_fft_pre_calc(uint16_t* buffer, int size, complex_t* data){
  float conv_values[size];

  for (int i = 0; i < size; i++){
    conv_values[i] = ADC_ADJUST(buffer[i]);
    data[i].r = conv_values[i];
    data[i].i = 0;    
  }

}

void dsp_fft(complex_t *data, int size) {
    if (size <= 1) return;

    int half = size / 2;
    complex_t even[half], odd[half];
    for (int i = 0; i < half; i++) {
        even[i] = data[2 * i];
        odd[i]  = data[2 * i + 1];
    }

    dsp_fft(even, half);
    dsp_fft(odd, half);

    for (int k = 0; k < half; k++) {
        float angle = -2 * PI * k / size;
        complex_t w = { cos(angle), sin(angle) };
        complex_t t = {
            w.r * odd[k].r - w.i * odd[k].i,
            w.r * odd[k].i + w.i * odd[k].r
        };
        data[k].r = even[k].r + t.r;
        data[k].i = even[k].i + t.i;
        data[k + half].r = even[k].r - t.r;
        data[k + half].i = even[k].i - t.i;
    }
}

void dsp_hann_window(complex_t *data, int size) {
    for (int i = 0; i < size; i++) {
        float w = 0.5 * (1 - cos(2 * PI * i / (size - 1)));
        data[i].r *= w;
        data[i].i *= w;
    }
}

float dsp_magnitude(complex_t c) {
    return sqrtf(c.r * c.r + c.i * c.i);
}

float dsp_find_peak(complex_t *data, int size, float bin) {
    int peak_index = 0;
    float max_mag = 0.0f;

    for (int i = 0; i < size/2 + 1; i++) {
        float mag = dsp_magnitude(data[i]);
        if (mag > max_mag) {
            max_mag = mag;
            peak_index = i;
        }
    }
    return (float)(peak_index)*bin;
}

bool check_note(float frequency){
  
  if (frequency < 75.0 || frequency > 350.0)
    return false;

  for (int i = E2; i <= E4; i++){
    if (fabs(frequency - strings[i].desired_freq)/strings[i].desired_freq  < DELTA_FREQ_FOR_TUNNING){
        current_string.note = i;
        current_string.desired_freq = strings[i].desired_freq;
        return true;
    }
 }
  return false;

}

// Callback da interrupção do botão
void gpio_callback(uint gpio, uint32_t events) {
    if (gpio == BTN_A_PIN) {
        static uint32_t last_interrupt_time = 0;
        uint32_t current_time = to_ms_since_boot(get_absolute_time());

        if (current_time - last_interrupt_time > DEBOUNCE_DELAY_MS) {
            last_interrupt_time = current_time;             
            button_pressed = true;
        }
    }
}

// Callback do timer 
static int64_t timer_callback(alarm_id_t id, void *user_data) {
    if (current_state == DETECTING)
        timer_expired = true;
    return 0; // Retorna 0 para não repetir
}

void set_LED_matrix(int tune){
    if (tune < VERY_DOWN || tune > VERY_UP)
        return;

    npClear();

    switch (tune){
        case VERY_DOWN:
            npSetLED(0, 128, 0, 0);
            npSetLED(1, 128, 0, 0);
            npSetLED(2, 128, 0, 0);
            npSetLED(3, 128, 0, 0);
            npSetLED(4, 128, 0, 0);
            npSetLED(5, 128, 0, 0);
            npSetLED(9, 128, 0, 0);
            npSetLED(10, 128, 0, 0);
            npSetLED(14, 128, 0, 0);
            npSetLED(15, 128, 0, 0);
            npSetLED(19, 128, 0, 0);
            npSetLED(20, 128, 0, 0);
            npSetLED(21, 128, 0, 0);
            npSetLED(22, 128, 0, 0);
            npSetLED(23, 128, 0, 0);
            npSetLED(24, 128, 0, 0);
            break;
        case DOWN:
            npSetLED(6, 128, 128, 0);
            npSetLED(7, 128, 128, 0);
            npSetLED(8, 128, 128, 0);
            npSetLED(11, 128, 128, 0);
            npSetLED(13, 128, 128, 0);
            npSetLED(16, 128, 128, 0);
            npSetLED(17, 128, 128, 0);
            npSetLED(18, 128, 128, 0);
            break;
        case TUNED:
            npSetLED(12, 0, 128, 0);
            npWrite();
            sleep_ms(250);
            npClear();

            npSetLED(6, 0, 128, 0);
            npSetLED(7, 0, 128, 0);
            npSetLED(8, 0, 128, 0);
            npSetLED(11, 0, 128, 0);
            npSetLED(13, 0, 128, 0);
            npSetLED(16, 0, 128, 0);
            npSetLED(17, 0, 128, 0);
            npSetLED(18, 0, 128, 0);

            npWrite();
            sleep_ms(250);

            npClear();

            for(int i = 0; i < NPLED_COUNT; i++){
                npSetLED(i, 0, 128, 0);
            }
            break;
        case UP:
            npSetLED(6, 128, 128, 0);
            npSetLED(7, 128, 128, 0);
            npSetLED(8, 128, 128, 0);
            npSetLED(11, 128, 128, 0);
            npSetLED(13, 128, 128, 0);
            npSetLED(16, 128, 128, 0);
            npSetLED(17, 128, 128, 0);
            npSetLED(18, 128, 128, 0);
            break;
        case VERY_UP:
            npSetLED(0, 128, 0, 0);
            npSetLED(1, 128, 0, 0);
            npSetLED(2, 128, 0, 0);
            npSetLED(3, 128, 0, 0);
            npSetLED(4, 128, 0, 0);
            npSetLED(5, 128, 0, 0);
            npSetLED(9, 128, 0, 0);
            npSetLED(10, 128, 0, 0);
            npSetLED(14, 128, 0, 0);
            npSetLED(15, 128, 0, 0);
            npSetLED(19, 128, 0, 0);
            npSetLED(20, 128, 0, 0);
            npSetLED(21, 128, 0, 0);
            npSetLED(22, 128, 0, 0);
            npSetLED(23, 128, 0, 0);
            npSetLED(24, 128, 0, 0);
    }

    npWrite();
}

