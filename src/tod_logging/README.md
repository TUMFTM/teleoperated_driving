# ToD Logging  {#tod_logging_docs}
the purpose of this repository is to have one repository that logs all debug and logging messages. the message type is diagnostic_msgs::msg::DignosticStatus. To log something, publish your debug message on the topic /Operator/Debug or /Operator/Logging


## Is this repo the repo you are looking for?
If you plan to extend the logger for additional functionalities, formating or filtering: yes!
If you plan publish debug/log data for your package in a different node: no!
If you plan publish log the framerate of the video and write a node that outputs the framerate metric: no!
Else: Look further in the repos that seem to include the functionality you are looking for

## Available Packages

### tod_logger
The purpose of this package is to bundle all available debug and logging infos within the entire ToD-System and provide it to the user or operator. 

## Doxygen Information
\version 1.0
\author TUM FTM
\defgroup tod_logging ToD Logging
\ingroup tod
\brief The purpose of this repository is to have one repository that logs all debug and logging messages.