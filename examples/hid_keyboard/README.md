This is an example of using AUSB to implement a HID keyboard device.

To build for an ESP32-S3 chip, assuming the `EDF_IDF` environment variable
points to your checkout of the ESP-IDF repository:

```
$ . "$(ESP_IDF)/export.sh"    # Run once for your shell instance
$ idf.py set-target esp32s3   # Run once for the project
$ idf.py menuconfig           # Run once for the project.
$ idf.py build
```
