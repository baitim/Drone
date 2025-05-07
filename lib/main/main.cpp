///server///
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "protocol_examples_utils.h"
#include "esp_tls_crypto.h"
#include <esp_http_server.h>
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_tls.h"
#include "esp_check.h"

#if !CONFIG_IDF_TARGET_LINUX
#include <esp_wifi.h>
#include <esp_system.h>
#include "nvs_flash.h"
#include "esp_eth.h"
#endif  // !CONFIG_IDF_TARGET_LINUX
///!server///

///motors///
#include <stdio.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sensors.hpp"
#include "drone.hpp"

#define EXAMPLE_HTTP_QUERY_KEY_MAX_LEN  (64)

constexpr const char* TAG = "example";

double TAR_OVERALL_THROTTLE = 0;
double TAR_YAW   = 0;
double TAR_PITCH = 0;
double TAR_ROLL  = 0;
int   THROTTLE_VERIFIED = 0;


double K_PROP[3] = {};
double K_DIFF[3] = {};
double K_INTG[3] = {};
bool  K_CHANGED = false;

esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err) {
    if (strcmp("/hello", req->uri) == 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "/hello URI is not available");
        /* Return ESP_OK to keep underlying socket open */
        return ESP_OK;
    } else if (strcmp("/echo", req->uri) == 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "/echo URI is not available");
        /* Return ESP_FAIL to close underlying socket */
        return ESP_FAIL;
    }
    /* For any other URI send 404 and close socket */
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Some 404 error message");
    return ESP_FAIL;
}

/* my custom handlers */
static esp_err_t control_post_handler(httpd_req_t *req) {
    char buf[100];
    int result, remaining = req->content_len;

    while (remaining > 0) {
        /* Read the data for the request */
        if ((result = httpd_req_recv(req, buf,
                        MIN(remaining, sizeof(buf)))) <= 0) {
            if (result == HTTPD_SOCK_ERR_TIMEOUT) {
                /* Retry receiving if timeout occurred */
                continue;
            }
            return ESP_FAIL;
        }

        /* Send back the same data */
        httpd_resp_send_chunk(req, buf, result);
        remaining -= result;
        
        double yaw_tmp, pitch_tmp, roll_tmp, throttle_tmp;
        int verification_tmp = 0;
        if (sscanf(buf, "%f %f %f %f %d", &yaw_tmp, &pitch_tmp, &roll_tmp, &throttle_tmp, &verification_tmp) == 5) {
            ESP_LOGI(TAG, "THROTTLE SET");
            TAR_OVERALL_THROTTLE = throttle_tmp;
            TAR_YAW = yaw_tmp;
            TAR_PITCH = pitch_tmp;
            TAR_ROLL = roll_tmp;
            THROTTLE_VERIFIED = verification_tmp;
        }

        /* Log data received */
        ESP_LOGI(TAG, "=========== RECEIVED DATA ==========");
        ESP_LOGI(TAG, "throttle: %.3f", throttle_tmp);
        ESP_LOGI(TAG, "[Y, P, R]: %.3f, %.3f, %.3f", yaw_tmp, pitch_tmp, roll_tmp);
        ESP_LOGI(TAG, "%.*s", result, buf);
        ESP_LOGI(TAG, "====================================");
    }

    // End response
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static const httpd_uri_t control_URI_handler = {
    .uri       = "/control",
    .method    = HTTP_POST,
    .handler   = control_post_handler,
    .user_ctx  = NULL
};

/* my custom handlers */
static esp_err_t PID_post_handler(httpd_req_t *req) {
    char buf[100];
    int result, remaining = req->content_len;

    while (remaining > 0) {
        /* Read the data for the request */
        if ((result = httpd_req_recv(req, buf,
                        MIN(remaining, sizeof(buf)))) <= 0) {
            if (result == HTTPD_SOCK_ERR_TIMEOUT) {
                /* Retry receiving if timeout occurred */
                continue;
            }
            return ESP_FAIL;
        }

        /* Send back the same data */
        httpd_resp_send_chunk(req, buf, result);
        remaining -= result;

        if (sscanf(buf, "%f %f %f %f %f %f %f %f %f", 
                                                    &K_PROP[0], &K_PROP[1], &K_PROP[2], 
                                                    &K_INTG[0], &K_INTG[1], &K_INTG[2],
                                                    &K_DIFF[0], &K_DIFF[1], &K_DIFF[2]) == 9) {
            ESP_LOGI(TAG, "GOT NEW COEFFS");
            K_CHANGED = true;
        } else {
            ESP_LOGE(TAG, "Invalid data in sscanf read");
        }

        /* Log data received */
        ESP_LOGI(TAG, "=========== RECEIVED DATA ==========");
        ESP_LOGI(TAG, "K_PROP: %f %f %f", K_PROP[0], K_PROP[1], K_PROP[2]);
        ESP_LOGI(TAG, "K_INTG: %f %f %f", K_INTG[0], K_INTG[1], K_INTG[2]);
        ESP_LOGI(TAG, "K_DIFF: %f %f %f", K_DIFF[0], K_DIFF[1], K_DIFF[2]);

        ESP_LOGI(TAG, "%.*s", result, buf);
        ESP_LOGI(TAG, "====================================");
    }

    // End response
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static const httpd_uri_t PID_URI_handler = {
    .uri       = "/PID",
    .method    = HTTP_POST,
    .handler   = PID_post_handler,
    .user_ctx  = NULL
};

static httpd_handle_t start_webserver(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
#if CONFIG_IDF_TARGET_LINUX
    // Setting port as 8001 when building for Linux. Port 80 can be used only by a privileged user in linux.
    // So when a unprivileged user tries to run the application, it throws bind error and the server is not started.
    // Port 8001 can be used by an unprivileged user as well. So the application will not throw bind error and the
    // server will be started.
    config.server_port = 8001;
#endif // !CONFIG_IDF_TARGET_LINUX
    config.lru_purge_enable = true;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &control_URI_handler);
        httpd_register_uri_handler(server, &PID_URI_handler);
        #if CONFIG_EXAMPLE_BASIC_AUTH
        httpd_register_basic_auth(server);
        #endif
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

#if !CONFIG_IDF_TARGET_LINUX
static esp_err_t stop_webserver(httpd_handle_t server) {
    // Stop the httpd server
    return httpd_stop(server);
}

static void disconnect_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server) {
        ESP_LOGI(TAG, "Stopping webserver");
        if (stop_webserver(*server) == ESP_OK) {
            *server = NULL;
        } else {
            ESP_LOGE(TAG, "Failed to stop http server");
        }
    }
}

static void connect_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data) {
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server == NULL) {
        ESP_LOGI(TAG, "Starting webserver");
        *server = start_webserver();
    }
}
#endif // !CONFIG_IDF_TARGET_LINUX


#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Разрешение ШИМ
#define LEDC_FREQUENCY          (50) // Частота ШИМ в Гц (стандартная для ESC)

#define MOTOR_COUNT 4
#define SENSOR_READ 1

#define BA_AWAIT 0

#if BA_AWAIT
static SemaphoreHandle_t ba_semaphore;

int custom_log_vprintf(const char *fmt, va_list args) {
    char log_msg[256];
    vsnprintf(log_msg, sizeof(log_msg), fmt, args);

    // Check if the log contains "<ba-add>"
    if (strstr(log_msg, "<ba-add>") != NULL) {
        // Release the semaphore
        xSemaphoreGive(ba_semaphore);
    }

    // Forward the log to the default output
    return vprintf(fmt, args);
}
#endif //BA_AWAIT

extern "C" void app_main() {
    static httpd_handle_t server = NULL;

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    /* Register event handlers to stop the server when Wi-Fi or Ethernet is disconnected,
     * and re-start it upon connection.
     */
    #if !CONFIG_IDF_TARGET_LINUX
    #ifdef CONFIG_EXAMPLE_CONNECT_WIFI
        ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));
    #endif // CONFIG_EXAMPLE_CONNECT_WIFI
    #ifdef CONFIG_EXAMPLE_CONNECT_ETHERNET
        ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &connect_handler, &server));
        ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ETHERNET_EVENT_DISCONNECTED, &disconnect_handler, &server));
    #endif // CONFIG_EXAMPLE_CONNECT_ETHERNET
    #endif // !CONFIG_IDF_TARGET_LINUX

    /* Start the server for the first time */
    server = start_webserver();
    // Initialize Wi-Fi and connect (as shown in the previous example)

    // Wait for the <ba-add> event
    #if BA_AWAIT
    ba_semaphore = xSemaphoreCreateBinary();
    esp_log_set_vprintf(custom_log_vprintf);
    if (xSemaphoreTake(ba_semaphore, portMAX_DELAY) == pdTRUE) {
        ESP_LOGI("main", "Detected <ba-add> event, continuing execution");
    }
    #endif

    TAR_OVERALL_THROTTLE = 100;
    double cur_overall_throttle = 100;

    /*
    4   13
     \ /
      ^
     / \
    15  32
    */

    drone_t drone;
    drone.initialize_motors_and_timer(LEDC_TIMER_13_BIT, LEDC_LOW_SPEED_MODE, 50, 1000, 2000, 4, 13, 32, 15);
    drone.set_duty(100);

    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI("DRONE",  "waiting for coeffs...");
    while (server) {
        if (K_CHANGED) {
            drone.set_PID(K_PROP, K_INTG, K_DIFF);
            K_CHANGED = false;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGI("DRONE",  "waiting for arming zero...");
    while (server) {
        if (TAR_OVERALL_THROTTLE == 0) {
            cur_overall_throttle = 0;
            drone.force_set_throttle(0);
            drone.update_motors();
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    drone.initialize_sensors();

    ESP_LOGI("DRONE", "GOT IT!, waiting 0.5s...");
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI("DRONE", "waiting done.");
    //motors armed

    double cur_tar_yaw   = TAR_YAW;
    double cur_tar_pitch = TAR_PITCH;
    double cur_tar_roll  = TAR_ROLL;

    drone.set_targets(TAR_YAW, TAR_PITCH, TAR_ROLL);

    while (server) {
        drone.read_sensors_values();

        if (TAR_YAW != cur_tar_yaw || TAR_PITCH != cur_tar_pitch || TAR_ROLL != cur_tar_roll) {
            cur_tar_yaw = TAR_YAW;
            cur_tar_pitch = TAR_PITCH;
            cur_tar_roll = TAR_ROLL;
            drone.set_targets(cur_tar_yaw, cur_tar_pitch, cur_tar_roll);
        }
        if ((TAR_OVERALL_THROTTLE != cur_overall_throttle) && THROTTLE_VERIFIED) {
            drone.set_throttle(TAR_OVERALL_THROTTLE);
            cur_overall_throttle = TAR_OVERALL_THROTTLE;
        }

        if (K_CHANGED) {
            drone.set_PID(K_PROP, K_INTG, K_DIFF);
            K_CHANGED = false;
        }

        if (THROTTLE_VERIFIED) {
            drone.processPID();
            drone.update_motors();    
        }
        
        drone.print_state();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
