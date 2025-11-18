# Using Throttle

## Connections

### Laptop
1. Connect to the teensy, both with ethernet and powerless usb
2. Make sure you are configured as a west project
3. Check proj.conf to make sure you have the right IP address
4. run west flash to send code to teensy
5. run "nc [insert ip] 19690" to be able to send commands
