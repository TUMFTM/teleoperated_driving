/**
 * @file virtual_input_device.hpp
 * @brief Defines a qml-based input device  
 * @copyright 2025 TUMFTM
 **/
 #pragma once
#include <QMainWindow>
#include "tod_input_devices/general/my_input_device.hpp"

namespace Ui {
    class JoystickWindow; ///< Forward declaration for UI class generated from Qt Designer.
}

namespace tod_input_device {

/**
 * @class VirtualInputDevice
 * @brief A virtual joystick implementation that simulates input device behavior in a graphical user interface.
 *
 * This class extends `QMainWindow` for GUI functionality and `MyInputDevice` for input device logic.
 * It provides virtual joystick capabilities, allowing users to simulate axis and button inputs through a GUI.
 * This way, we are able to steer the vehicle without providing a real physical device for steering and 
 * accelerating the vehicle.
 */
class VirtualInputDevice : public QMainWindow, public MyInputDevice {
    Q_OBJECT

public:
    /**
     * @brief Constructs a VirtualInputDevice instance.
     * @param axisCb Callback function for axis value changes.
     * @param buttonCb Callback function for button state changes.
     * @param parent Pointer to the parent QWidget (default: nullptr).
     *
     * Initializes the virtual input device UI and sets up signal-slot connections for handling user input.
     */
    explicit VirtualInputDevice(std::function<void(const int, const double)> axisCb,
                                std::function<void(const int, const int)> buttonCb,
                                QWidget *parent = nullptr);

    /**
     * @brief Destructor for the VirtualInputDevice class.
     *
     * Cleans up UI resources and closes the window.
     */
    ~VirtualInputDevice();

    /**
     * @brief Activates the virtual joystick.
     * @return `true` if activation is successful.
     *
     * Emits the `activateSignal` and makes the joystick visible.
     */
    bool activate() override;

    /**
     * @brief Deactivates the virtual joystick.
     * @return `true` if deactivation is successful.
     *
     * Emits the `deactivateSignal` and hides the joystick.
     */
    bool deactivate() override;

    /**
     * @brief Terminates the virtual joystick.
     *
     * Emits the `terminateSignal` and quits the application.
     */
    void terminate() override;

private slots:
    /**
     * @brief Slot for handling user input events from the GUI.
     * @param x X-axis value of the input.
     * @param y Y-axis value of the input.
     *
     * Invokes the axis callback with the provided input values.
     */
    void on_userInput(double x, double y);

    /**
     * @brief Slot for activating the virtual joystick.
     *
     * Makes the joystick window visible and sets the `running` state to `true`.
     */
    void on_activate();

    /**
     * @brief Slot for deactivating the virtual joystick.
     *
     * Hides the joystick window and sets the `running` state to `false`.
     */
    void on_deactivate();

    /**
     * @brief Slot for terminating the virtual joystick.
     *
     * Closes the application gracefully.
     */
    void on_terminate();

signals:
    void activateSignal();   ///< Signal emitted to activate the virtual joystick.
    void deactivateSignal(); ///< Signal emitted to deactivate the virtual joystick.
    void terminateSignal();  ///< Signal emitted to terminate the virtual joystick.

private:
    Ui::JoystickWindow *ui;  ///< Pointer to the UI elements of the joystick window.

    /**
     * @brief Event filter for handling key press and release events.
     * @param target The object receiving the event.
     * @param e The event being processed.
     * @return `true` if the event was handled, `false` otherwise.
     *
     * Captures key press/release events to trigger button callbacks.
     */
    bool eventFilter(QObject *target, QEvent *e);

    /**
     * @brief Handles window resize events.
     * @param event The resize event object.
     *
     * Adjusts the geometry of the joystick UI elements to fit the new window size.
     */
    void resizeEvent(QResizeEvent *event);
};

} // namespace tod_input_device
