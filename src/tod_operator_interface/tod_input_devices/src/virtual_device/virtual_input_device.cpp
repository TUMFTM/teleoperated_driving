/**
 * @file virtual_input_device.cpp
 * @brief Defines a qml-based input device  
 * @copyright 2025 TUMFTM
 **/
#include <QtQuick/QQuickView>
#include <QtQuick/QQuickItem>
#include <QUrl>
#include <QCoreApplication>
#include "virtual_input_device.hpp"
#include "ui_joystickwindow.h"

namespace tod_input_device {

VirtualInputDevice::VirtualInputDevice(std::function<void(const int, const double)> axisCb,
    std::function<void(const int, const int)> buttonCb, QWidget *parent)
    : QMainWindow(parent), MyInputDevice(axisCb, buttonCb) {

    ui = new Ui::JoystickWindow;
    ui->setupUi(this);

    // Set the QML source for the virtual joystick.
    QString path_prefix = QCoreApplication::applicationDirPath();
    ui->quickWidget->setSource(QUrl(path_prefix + "/virtual_joystick.qml"));

    // Connect the QML signal to the on_userInput slot.
    connect(ui->quickWidget->rootObject(), SIGNAL(userInput(double, double)),
        this, SLOT(on_userInput(double, double)));

    // Connect signals to corresponding slots.
    connect(this, &VirtualInputDevice::activateSignal, this, &VirtualInputDevice::on_activate);
    connect(this, &VirtualInputDevice::deactivateSignal, this, &VirtualInputDevice::on_deactivate);
    connect(this, &VirtualInputDevice::terminateSignal, this, &VirtualInputDevice::on_terminate);
    
    installEventFilter(this);
}

VirtualInputDevice::~VirtualInputDevice() {
    delete ui;
    this->close();
}

bool VirtualInputDevice::activate() {
    emit activateSignal();
    return true;
}

bool VirtualInputDevice::deactivate() {
    emit deactivateSignal();
    return true;
}

void VirtualInputDevice::terminate() {
    emit terminateSignal();
}

void VirtualInputDevice::on_userInput(double x, double y) {
    axis_callback(0, -x);
    axis_callback(1, y);
}

void VirtualInputDevice::on_activate() {
    this->show();
    running = true;
}

void VirtualInputDevice::on_deactivate() {
    this->hide();
    running = false;
}

void VirtualInputDevice::on_terminate() {
    QApplication::quit();
}

bool VirtualInputDevice::eventFilter(QObject *target, QEvent *event) {
    (void)target;  // Unused parameter 
    if (event->type() == QEvent::KeyPress) {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
        if (!keyEvent->isAutoRepeat()) {
            button_callback(keyEvent->key(), 1);
            ui->pressedButton->setText("Button: " + QString::number(keyEvent->key()));
        }
        return true;
    }
    if (event->type() == QEvent::KeyRelease) {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
        if (!keyEvent->isAutoRepeat()) {
            button_callback(keyEvent->key(), 0);
            ui->pressedButton->setText("Button: ");
        }
        return true;
    }
    return false;
}

void VirtualInputDevice::resizeEvent(QResizeEvent *event) {
    static int xOffset{10};
    static int yOffset{10};
    ui->quickWidget->setGeometry(xOffset, yOffset,
        event->size().width() - 2 * xOffset,
        event->size().height() - 2 * yOffset - ui->pressedButton->height());
    ui->pressedButton->move(0, event->size().height() - ui->pressedButton->height());
}

} // namespace tod_input_device
