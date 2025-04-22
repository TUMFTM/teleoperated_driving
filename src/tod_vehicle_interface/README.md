# tod_vehicle_interface
This repository  provides a set packages with
  * configuration files for each available vehicle (`tod_vehicle_config` package)
  * along with the vehicle bridge packages that establish the interface with the vehicle.

To create a new vehicle interface to the following:
  * Create a new vehicle configuration folder inside the 
    `tod_vehicle_config/vehicle_config/*`. Use a provided template
    samples to match the required structure and naming scheme.
  * Create a new vehicle bridge (`tod_*_bridge`) and place your nodes in a way 
    that will match the ToD software stack topic requirements (`/Vehicle/Bridge/...`).
For a complete set of requirements please refer to provided demo samples (more coming soon!).

## Sample Interfaces
* RC Car (F1TENTH)
* *More coming soon...*
