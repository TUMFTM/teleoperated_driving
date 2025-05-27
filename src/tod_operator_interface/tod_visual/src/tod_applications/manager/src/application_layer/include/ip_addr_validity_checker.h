// Copyright 2020 Feiler
/**
 * @file ip_addr_validity_checker.hpp
 * @brief Provides an interface for validating IP addresses.
 *
 * This file defines the `IpAddrValidityChecker` class, which serves as a base interface for
 * checking the validity of IP addresses.
 */

#pragma once
#include <string>


/**
 * @class IpAddrValidityChecker
 * @brief An abstract class for validating IP addresses.
 *
 * This class defines a virtual interface for IP address validation. Derived classes should implement
 * the `validate` method to provide custom validation logic.
 */
class IpAddrValidityChecker {
public:
    virtual bool validate(const std::string& ip_addr) = 0;
    virtual ~IpAddrValidityChecker() = default;
};
