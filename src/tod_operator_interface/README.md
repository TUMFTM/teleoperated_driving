# tod_operator_interface
Packages that either display data to the operator or offer an interface that the operator can interact with.

the purpose of this repository is to have one common repository for the /Operator/Visual/... namespaces in the tod code.
Core functionality of the repo are:

- to send/receive data to/from tod_visual
- to configure the video streams using a gui that sends/receives the VideoConfig
- to update the operator state using a gui
- to read in the input of an operator via usb/joystick devices


## Is this repo the repo you are looking for?
- If you plan to have a gui for the operator to interact with: yes!
- If you plan to have get input from a specific device from the operator: yes!
- If you plan to process Operator input: no!
- If you plan to preprocess something prior to display it, e.g. projections: no!
Else: Look further in the repos that seem to include the functionality you are looking for