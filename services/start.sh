#/bin/sh

sudo systemctl start usbcamera.service
sudo systemctl start mavlink-router.service

sudo systemctl status usbcamera.service
sudo systemctl status mavlink-router.service
