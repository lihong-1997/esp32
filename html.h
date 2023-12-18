#ifndef HTML_H
#define HTML_H


#define ROOT_HTML_1 "<!DOCTYPE html> \
<html>\
    <head>\
        <title>ESP32配置</title>\
        <meta charset=\"UTF-8\" name=\"viewport\" content=\"width=device-width, initial-scale=1\">\
    </head>\
    <style type=\"text/css\">\
        body{font: normal 14px/100% \"Andale Mono\", AndaleMono, monospace;background-color: #4F6877;color: #fff;}\
        .input{display: flex;\
            justify-content: center;\
            align-items: center; margin-top: 10px;}\
        .input span{width: 100px; float: left; float: left; height: 36px; line-height: 36px;}\
        .input input{height: 30px;width: 200px;outline: none;}\
        .input select{height: 36px;width: 208px;outline: none;}\
        .btn{font-weight: bold;max-width: 100px; height: 45px; background-color: #6BBE92; border:0px; color:#ffffff; margin-top:15px;}\
    </style>\
    <body>\
        <form method=\"POST\" action=\"configwifi\" style=\"margin-top: 200px;text-align:center;\">\
                <span style=\"font-size:24px;\">\
                    Monitor 配置页面\
                </span>\
                <label class=\"input\">\
                    <span>\
                    </span>\
                </label>\
            <label class=\"input\">\
                <span>\
                    User Name\
                </span>\
                <input type=\"text\" name=\"name\">\
            </label>\
            <label class=\"input\">\
                <span>\
                    WiFi SSID\
                </span>\
                <select id=\"ssid\" name=\"ssid\">"


#define ROOT_HTML_2   "</select>\
                  </label>\
                  <label class=\"input\">\
                      <span>\
                          WiFi PASS\
                      </span>\
                      <input type=\"text\"  name=\"pass\">\
                  </label>\
                  <label class=\"input\">\
                      <input class=\"btn\" type=\"submit\" name=\"submit\" value=\"Submit\">\
                  </label>\
              </form>\
          </body>\
      </html>"

#endif