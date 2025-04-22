# tod_safety (#tod_safety)

The purpose of this repository is to have one common repository for the /operator/safety/... and /vehicle/safety/... namespaces in the tod code.
Core functionality of this repository is a safety gate that passes the control commands to the actuation depending on /operator/monitoring/... and /vehicle/monitoring/... topics.
The package can be extended by additional safety packages.

## Available Packages
- tod_safety_gate

## Is this repo the repo you are looking for?
If you plan to apply any safety functionalities: yes!
If you plan to monitor anything to take according safety actions: no!

## Doxygen stuff
\version 1.0
\author Florian Pfab
\defgroup tod_safety ToD Safety
\ingroup tod
\brief safety features of the teleoperation system