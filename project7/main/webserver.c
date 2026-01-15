// Functions to run a simple webserver. This will also connect to wifi and
// start any other necessary services like mDNS and NTP

#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <esp_event.h>
#include <esp_wifi.h>
#include <esp_http_server.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <mdns.h>
#include <esp_netif_sntp.h>
#include <esp_sntp.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <stdio.h>
#include <time.h>

#include "webserver.h"

/**
 * External functions used to get data
 */
extern double dhtGetTemperature();
extern double dhtGetHumidity();
extern double thermostatGetSetpoint();
extern bool thermostatGetStatus();
extern void schedulerGetSchedule(int period, struct tm *schedTime, double *schedSetpoint);
extern void schedulerSetSchedule(int period, struct tm time, double setpoints);
extern void dataloggerGetLoggedData(int period, double *temperature,
                                    double *humidity, double *setpoint, bool *status);

#define WIFI_SSID CONFIG_ESP_WIFI_SSID
#define WIFI_PASSWORD CONFIG_ESP_WIFI_PASSWORD
#define HOSTNAME CONFIG_ESP_HOSTNAME

static const char *TAG = "WebServer";

// our webserver
static httpd_handle_t server = NULL;

// forward declarations of internal functions
static esp_err_t wifiInit(void);
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data);

static esp_err_t nvsInit(void);
static esp_err_t netifInit(void);
static esp_err_t mdnsInit(void);

static esp_err_t ntpInit(void);

static esp_err_t startWebServer(void);
static esp_err_t handleRootPage(httpd_req_t *req);
static esp_err_t handleSetSchedulePage(httpd_req_t *req);
static esp_err_t handleDrawDataloggerGraph(httpd_req_t *req);

// Connect to wifi and start the web server
esp_err_t webServerInit()
{
    // connect to wifi. Once conencted, web server will be started in event handler
    ESP_RETURN_ON_ERROR(wifiInit(), TAG, "Cannot connect to wifi");

    return ESP_OK;
}

// Connect to WiFi
esp_err_t wifiInit(void)
{
    // non-volatile storage used for wifi config
    ESP_RETURN_ON_ERROR(nvsInit(), TAG, "Cannot initialize NVS");

    // event loop to handle wifi and netif events
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "Cannot create event loop");

    // network interface
    ESP_RETURN_ON_ERROR(netifInit(), TAG, "Cannot initialize network interface");

    // start MDNS service
    ESP_RETURN_ON_ERROR(mdnsInit(), TAG, "Cannot initialize mDNS");

    // initialize NTP to get correct time
    ESP_RETURN_ON_ERROR(ntpInit(), TAG, "Cannot initialize NTP");

    // configure wifi driver
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    return ESP_OK;
}

// Handler for WiFi and IP events. Once connected, we start our webserver.
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    // once driver is started, try to connect to wifi
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }

    // if we've disconnected, stop the webserver and try to reconnect
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGI(TAG, "Disconnected from WiFi access point");

        // stop webserver if running
        if (server != NULL)
        {
            ESP_LOGI(TAG, "Stopping webserver");
            httpd_stop(server);
            server = NULL;
        }
        ESP_LOGI(TAG, "Reconnecting to WiFi access point");
        esp_wifi_connect();
    }

    // if we're connected, then start webserver
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Connected ip:" IPSTR, IP2STR(&event->ip_info.ip));

        // start webserver if not already running
        if (server == NULL)
        {
            ESP_LOGI(TAG, "Starting webserver");
            ESP_ERROR_CHECK(startWebServer());
        }
    }
}

// Setup NVS
static esp_err_t nvsInit(void)
{
    // initialize NVS (needed for wifi config data)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // try to erase and then initialize again
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    return ret;
}

// Initialiaze the netif interface, which provides TCP/IP support
static esp_err_t netifInit()
{
    ESP_ERROR_CHECK(esp_netif_init());
    esp_netif_create_default_wifi_sta();

    return ESP_OK;
}

// Initialize the mDNS service, configuring the hostname and instance name
static esp_err_t mdnsInit(void)
{

    // initialize mDNS
    ESP_ERROR_CHECK(mdns_init());

    // set hostname
    ESP_ERROR_CHECK(mdns_hostname_set(HOSTNAME));
    ESP_LOGI(TAG, "mdns hostname set to [%s]", HOSTNAME);

    // set default instance name
    ESP_ERROR_CHECK(mdns_instance_name_set("ESP32 mDNS"));

    mdns_txt_item_t serviceTxtData[3] = {
        {"board", "{esp32}"},
        {"u", "user"},
        {"p", "password"}};

    // initialize mdns service for web server
    ESP_ERROR_CHECK(mdns_service_add("ESP32 Webserver", "_http", "_tcp", 80, serviceTxtData, 3));

    return ESP_OK;
}

// Connect to NTP server to get correct time
static esp_err_t ntpInit(void)
{
    // configure NTP time
    esp_sntp_config_t sntp_config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");

    // initialize sntp
    ESP_ERROR_CHECK(esp_netif_sntp_init(&sntp_config));

    // set the local timezone
    setenv("TZ", "EST+5EDT,M3.2.0/2,M11.1.0/2", 1);
    tzset();

    return ESP_OK;
}


static const httpd_uri_t setSchedulePage = {
    .uri = "/updateschedule",
    .method = HTTP_POST,
    .handler = handleSetSchedulePage,
    .user_ctx = NULL,
};

static const httpd_uri_t rootPage = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = handleRootPage,
    .user_ctx = NULL,
};

static const httpd_uri_t drawDataloggerGraph = {
    .uri = "/temperature.svg",
    .method = HTTP_GET,
    .handler = handleDrawDataloggerGraph,
    .user_ctx = NULL,
};

// Start the webserver
static esp_err_t startWebServer(void)
{
    // configure http server
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    // start the server
    ESP_LOGI(TAG, "Starting webserver on port %d", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK)
    {
        // set URI handlers
        ESP_LOGI(TAG, "Registering web page handlers");
        httpd_register_uri_handler(server, &rootPage);
        httpd_register_uri_handler(server, &setSchedulePage);
        httpd_register_uri_handler(server, &drawDataloggerGraph);
    }

    return ESP_OK;
}

// Handler for root web page
static esp_err_t handleRootPage(httpd_req_t *req)
{
    static char message[4096];
    time_t currentTime = time(NULL);
    struct tm *local_tm = localtime(&currentTime);

    // calculate uptime (seconds since boot)
    time_t uptime = esp_timer_get_time() / 1000000;

    // open a stream to message buffer
    FILE *stream = fmemopen(message, 4096, "w");

    // print html to stream
    fprintf(stream, "<!DOCTYPE html>\n");
    fprintf(stream, "<html>\n");
    fprintf(stream, "<head>\n");
    //    fprintf(stream, "<meta http-equiv='refresh' content='30'>\n");
    fprintf(stream, "<title>Smart Thermostat</title>\n");
    fprintf(stream, "<style>\n");
    fprintf(stream, "* { box-sizing: border-box; }\n");
    fprintf(stream, "body { background-color: white; font-family: Arial, Verdana, Tahoma, sans-serif; font-size: 100%%; }\n");
    fprintf(stream, ".header, .footer {background-color: grey; color: white; padding: 15px; }\n");
    fprintf(stream, ".container { display: flex; justify-content: space-between; }\n");
    fprintf(stream, ".column { float: left; padding: 15px; width: 33%%; text-align: center; }\n");
    fprintf(stream, ".column2 { float: left; padding: 50px; width: 67%%; text-align: center; }\n");
    fprintf(stream, ".clearfix::after { content: \"\"; clear: both; display: table; }\n");
    fprintf(stream, "@media screen and (max-width: 600px) { .column {width: 100%%; } }\n");
    fprintf(stream, "h1 { font-size: 2.0em; font-weight: normal; }\n");
    fprintf(stream, "h2.a { font-size: 1.5em; font-weight: normal; }\n");
    fprintf(stream, "p { font-size: 1.0em; font-weight: normal; }\n");
    fprintf(stream, "p.a { font-size: 3.0em; font-weight: bold; }\n");
    fprintf(stream, "table { font-family: arial, sans-serif,; border-collapse: collapse; width: 100%%; }\n");
    fprintf(stream, "td, th { border: 1px solid #dddddd; text-align: left; padding: 8px; }\n");
    fprintf(stream, "tr:nth-child(even) { background-color: #dddddd; }\n");
    fprintf(stream, "\n");
    fprintf(stream, "</style> </head>\n");
    fprintf(stream, "<body>\n");
    fprintf(stream, "<div class=\"container\">\n");
    fprintf(stream, "<div>%s</div>\n", asctime(local_tm));
    fprintf(stream, "<div>Uptime: %lld days, %02lld:%02lld:%02lld</div>\n",
            uptime / 86400,
            (uptime % 86400) / 3600,
            (uptime % 3600) / 60,
            uptime % 60);
    fprintf(stream, "</div>\n");
    fprintf(stream, "<div class=\"header\"></div>\n");

    // display temperature, humidity, and current setpoint
    fprintf(stream, "<div class=\"clearfix\">\n");
    fprintf(stream, "   <div class=\"column\"><h2>Temperature</h2><p class='a'>%4.1f&#8457</p></div>\n",
            dhtGetTemperature());
    fprintf(stream, "   <div class=\"column\"><h2>Humidity</h2><p class='a'>%4.1f%%</p></div>\n",
            dhtGetHumidity());
    if (thermostatGetStatus())
    {
        // heat is on
        fprintf(stream, "   <div class=\"column\"><h2>Setting (Heat ON)</h2><p class='a'>%4.1f&#8457</p></div>\n",
                thermostatGetSetpoint());
    }
    else
    {
        // heat is off
        fprintf(stream, "   <div class=\"column\"><h2>Setting (Heat OFF)</h2><p class='a'>%4.1f&#8457</p></div>\n",
                thermostatGetSetpoint());
    }
    fprintf(stream, "</div>\n");
    fprintf(stream, "<div class=\"footer\"></div>\n");
    fprintf(stream, "<div class=\"clearfix\">\n");
    fprintf(stream, "<div class=\"column2\">\n");

    // Include graph of last 24 hours
    fprintf(stream, "<h1>Last 24 Hours</h1>\n");
    fprintf(stream, "<img src=\"temperature.svg\" alt=\"24 Hour Temperature Graph\" style=\"width:100%%\" />\n");
    fprintf(stream, "</div>\n");

    // display the schedule
    char *table_labels[4] = {"Wake", "Leave", "Return", "Sleep"};
    char *time_labels[4] = {"wake-time", "leave-time", "return-time", "sleep-time"};
    char *heat_labels[4] = {"wake-heat", "leave-heat", "return-heat", "sleep-heat"};

    fprintf(stream, "<div class=\"column\">\n");
    fprintf(stream, "   <h1>Set schedule times and temperatures:</h1>\n");
    fprintf(stream, "   <form action=\"/updateschedule\" method=\"post\">\n");
    fprintf(stream, "   <table>\n");
    fprintf(stream, "      <tr>\n");
    fprintf(stream, "         <th>Period</th>\n");
    fprintf(stream, "         <th>Time</th>\n");
    fprintf(stream, "         <th>Heat Setting</th>\n");
    fprintf(stream, "      </tr>\n");

    struct tm schedTime;
    double schedSetpoint;

    // create table of times and heat setpoints
    for (int i = 0; i < 4; i++)
    {
        schedulerGetSchedule(i, &schedTime, &schedSetpoint);

        fprintf(stream, "      <tr>\n");
        fprintf(stream, "         <td>%s</td>\n", table_labels[i]);
        fprintf(stream, "            <td><input type=\"time\" name=\"%s\" value=\"%02d:%02d\"/></td>\n",
                time_labels[i], schedTime.tm_hour, schedTime.tm_min);
        fprintf(stream, "         <td><input type=\"number\" id=\"%s\" name=\"%s\" min=\"55\" max=\"85\" value=\"%d\"/></td>\n",
                heat_labels[i], heat_labels[i], (int)schedSetpoint);
        fprintf(stream, "      </tr>\n");
    }

    fprintf(stream, "   </table>\n");
    fprintf(stream, "   <br>\n");
    fprintf(stream, "   <input type=\"submit\" value=\"Update Schedule\">\n");
    fprintf(stream, "   </form>\n");

    fprintf(stream, "</div>\n");
    fprintf(stream, "</div>\n");

    fprintf(stream, "</body>\n");
    fprintf(stream, "</html>\n");
    fclose(stream);

    // send response
    httpd_resp_send(req, message, HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

// Handler for set schedule web page
static esp_err_t handleSetSchedulePage(httpd_req_t *req)
{
    struct tm time;
    double setpoint;
    char *time_label[4] = {"wake-time", "leave-time", "return-time", "sleep-time"};
    char *heat_label[4] = {"wake-heat", "leave-heat", "return-heat", "sleep-heat"};

    char timebuf[40];
    char tempbuf[40];
    char rbuf[1000];
    httpd_req_recv(req, rbuf, 1000);

    // try to get all 4 sets of values
    for (int i = 0; i < 4; ++i)
    {
        // get *-time args
        if ((httpd_query_key_value(rbuf, time_label[i], timebuf, 40) == ESP_OK) && (httpd_query_key_value(rbuf, heat_label[i], tempbuf, 40) == ESP_OK))
        {
            // parse the time -- note the : is url-encoded as %3A
            sscanf(timebuf, "%d%%3A%d", &time.tm_hour, &time.tm_min);

            // parse the setpoint
            sscanf(tempbuf, "%lf", &setpoint);

            // update the scheduler
            schedulerSetSchedule(i, time, setpoint);
        }
    }

    // send response to httpd server
    static char message[200];
    FILE *stream = fmemopen(message, 4096, "w");

    fprintf(stream, "<!DOCTYPE html>\n");
    fprintf(stream, "<html><head><meta http-equiv=\"refresh\" content=\"0; url=/\" /></head>\n");
    fprintf(stream, "<body><p>\"Updating schedule...\"</p></body>\n");
    fprintf(stream, "</html>\n");

    fclose(stream);

    // send response
    httpd_resp_send(req, message, HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

// Handler to draw the graph of 24-hours of data
static esp_err_t handleDrawDataloggerGraph(httpd_req_t *req)
{
    static char message[4 * 1024];

    // open a stream to message buffer
    FILE *stream = fmemopen(message, 4 * 1024, "w");

    fprintf(stream, "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" width=\"600\" height=\"300\">\n");
    fprintf(stream, "<rect width=\"600\" height=\"300\" fill=\"rgb(250, 230, 210)\" stroke-width=\"1\" stroke=\"rgb(0,0,0)\" />\n");

    // draw the axes
    fprintf(stream, "\n<g id=\"axes\" stroke=\"black\">\n\n");

    // y-axis -- 240 pixels long (row 10 to 250), temperatures 55 to 85 (rows 250 to 10), 8 pixels per degree
    fprintf(stream, "<g id=\"y-axis\">\n");
    fprintf(stream, "<line x1=\"60\" y1=\"10\" x2=\"60\" y2=\"250\" stroke-width=\"1\" />\n");
    fprintf(stream, "<line x1=\"60\" y1=\"10\" x2=\"60\" y2=\"250\" stroke-width=\"10\" stroke-dasharray=\"1,39\" />\n");
    fprintf(stream, "<line x1=\"60\" y1=\"50\" x2=\"60\" y2=\"250\" stroke-width=\"16\" stroke-dasharray=\"1,79\" />\n");

    // y-axis labels
    fprintf(stream, "<text x=\"20\" y=\"57\" font-size=\"20\" fill=\"black\">80&#176;</text>\n");
    fprintf(stream, "<text x=\"20\" y=\"137\" font-size=\"20\" fill=\"black\">70&#176;</text>\n");
    fprintf(stream, "<text x=\"20\" y=\"217\" font-size=\"20\" fill=\"black\">60&#176;</text>\n");
    fprintf(stream, "</g>\n"); // end of y-axis

    // x-axis
    fprintf(stream, "<g id=\"x-axis\">\n");
    fprintf(stream, "<line x1=\"60\" y1=\"250\" x2=\"540\" y2=\"250\" stroke-width=\"1\" />\n");
    fprintf(stream, "<line x1=\"60\" y1=\"250\" x2=\"540\" y2=\"250\" stroke-width=\"10\" stroke-dasharray=\"1,19\" />\n");
    fprintf(stream, "<line x1=\"60\" y1=\"250\" x2=\"540\" y2=\"250\" stroke-width=\"16\" stroke-dasharray=\"1,59\" />\n");

    // x-axis labels every 3 hrs
    for (int hr = 0; hr < 24; hr += 3)
    {
        // do we print am/pm?
        if ((hr % 12) == 0)
        {
            // 12am or 12pm -- we print am/pm
            fprintf(stream, "<text x=\"%d\" y=\"270\" font-size=\"15\" fill=\"black\">12%2s</text>\n",
                    (45 + 20 * hr), (hr < 12) ? "am" : "pm");
        }
        else
        {
            // other times, just print hour
            fprintf(stream, "<text x=\"%d\" y=\"270\" font-size=\"15\" fill=\"black\">%d</text>\n",
                    (55 + 20 * hr), (hr % 12));
        }
    }
    fprintf(stream, "</g>\n\n"); // end of x-axis
    fprintf(stream, "</g>\n");   // end of axes

    // graph temperature and setpoints
    // first, let's get a snapshot of the temperature and setpoint data so we don't have to
    // ask for it twice (have to graph all the setpoint data first, then the temperature)
    double temperature[24 * 4];
    double setpoint[24 * 4];

    for (int i = 0; i < 24 * 4; ++i)
    {
        double humidity; // placeholders
        bool status;

        dataloggerGetLoggedData(i, &(temperature[i]), &humidity, &(setpoint[i]), &status);
    }

    // figure out index of current time so we can have a gap in the graph
    time_t rawtime;
    struct tm *now;
    time(&rawtime);
    now = localtime(&rawtime);
    int nowIndex = (now->tm_hour * 60 + now->tm_min) / 15;

    // graph the setpoint data first. We graph from 0 to now, skip 1, and then now+1 to 95
    fprintf(stream, "<g id=\"setpoint\" >\n");
    if (nowIndex > 0)
    {
        fprintf(stream, "<polyline fill=\"none\" stroke=\"green\" stroke-width=\"1\" points=\"\n");
        for (int i = 0; i <= nowIndex; ++i)
        {
            int x = i * 5 + 60;
            int y = 690 - (int)(8 * setpoint[i]);

            // make sure it's on the graph
            if ((setpoint[i] >= 55) && (setpoint[i] <= 85))
                fprintf(stream, "%d,%d\n", x, y);
        }
        fprintf(stream, "\" />\n");
    }

    if (nowIndex < 4 * 24 - 2)
    {
        fprintf(stream, "<polyline fill=\"none\" stroke=\"green\" stroke-width=\"1\" points=\"\n");
        for (int i = nowIndex + 1; i < 4 * 24; ++i)
        {
            int x = i * 5 + 60;
            int y = 690 - (int)(8 * setpoint[i]);

            // make sure it's on the graph
            if ((setpoint[i] >= 55) && (setpoint[i] <= 85))
                fprintf(stream, "%d,%d\n", x, y);
        }
        fprintf(stream, "\" />\n");
    }
    fprintf(stream, "</g>\n\n");

    // now the temperature
    fprintf(stream, "<g id=\"temperature\" >\n");
    if (nowIndex > 0)
    {
        fprintf(stream, "<polyline fill=\"none\" stroke=\"red\" stroke-width=\"1\" points=\"\n");
        for (int i = 0; i <= nowIndex; ++i)
        {
            int x = i * 5 + 60;
            int y = 690 - (int)(8 * temperature[i]);

            // make sure it's on the graph
            if ((temperature[i] >= 55) && (temperature[i] <= 85))
                fprintf(stream, "%d,%d\n", x, y);
        }
        fprintf(stream, "\" />\n");
    }

    if (nowIndex < 4 * 24 - 2)
    {
        fprintf(stream, "<polyline fill=\"none\" stroke=\"red\" stroke-width=\"1\" points=\"\n");
        for (int i = nowIndex + 1; i < 4 * 24; ++i)
        {
            int x = i * 5 + 60;
            int y = 690 - (int)(8 * temperature[i]);

            // make sure it's on the graph
            if ((temperature[i] >= 55) && (temperature[i] <= 85))
                fprintf(stream, "%d,%d\n", x, y);
        }
        fprintf(stream, "\" />\n");
    }
    fprintf(stream, "</g>\n\n");

    fprintf(stream, "</svg>\n");

    fclose(stream);

    // response is an svg file
    httpd_resp_set_type(req, "image/svg+xml");

    // send the response
    httpd_resp_send(req, message, HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}
