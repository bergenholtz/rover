#/bin/sh

sudo systemctl stop usbcamera.service
sudo systemctl stop mavlink-router.service

sudo systemctl status usbcamera.service
sudo systemctl status mavlink-router.service
