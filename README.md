# vaccibox

waitUntilResponse(); helper function was taken from https://github.com/CheapskateProjects/SimGpsTransmitter


The code sets up a system that collects GPS coordinates and temperature data using an Arduino board and a GSM module (SIM800L). It establishes a GPRS connection to enable internet access through the SIM card, retrieves GPS data, and sends it along with the temperature data to a remote web server at a specified interval. If errors occur with the GPS module or GSM module, corresponding LEDs will light up, and if the number of errors exceeds a certain threshold, the system will reboot. The code also includes functions for converting float values to strings and reading responses from the GSM module.
