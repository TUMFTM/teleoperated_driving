// Copyright 2020 Feiler
/**
 * @file ipv4_validity_checker.h
 * @brief Concrete class for IPv4 address validation.
 *
 * This file defines the `IPv4ValidityChecker` class, which implements the `IpAddrValidityChecker` interface
 * for validating IPv4 addresses.
 */
#pragma once
#include "ip_addr_validity_checker.h"
#include <string>
#include <arpa/inet.h>

/**
 * @class IPv4ValidityChecker
 * @brief A concrete implementation for validating IPv4 addresses.
 *
 * The `IPv4ValidityChecker` class provides functionality to verify whether a given string is a valid IPv4 address.
 * It extends the abstract base class `IpAddrValidityChecker` and implements the `validate` method.
 */
class IPv4ValidityChecker : public IpAddrValidityChecker {
public:
    IPv4ValidityChecker();
    bool validate(const std::string& ip_addr) override;
};
