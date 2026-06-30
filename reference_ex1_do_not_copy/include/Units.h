#pragma once

// Units.h
// Shared constants and unit aliases for the Drone Mapper project.

// --------------------
// mp-units includes
// --------------------

#include <mp-units/systems/si.h>
#include <mp-units/systems/angular.h>
#include <mp-units/framework/quantity.h>


// --------------------
// Cell values
// --------------------

constexpr int UNKNOWN_CELL = -1;
constexpr int FREE_CELL = 0;
constexpr int OCCUPIED_CELL = 1;
constexpr int OUT_OF_BOUNDS = -2;
constexpr int SUPPORTED_RESOLUTION_CM = 1;


// --------------------
// Unit symbols
// --------------------

using mp_units::si::unit_symbols::cm;
using mp_units::si::unit_symbols::m;
using mp_units::angular::unit_symbols::deg;


// --------------------
// Strong unit aliases
// --------------------

using Cm = decltype(1 * cm);
using Meter = decltype(1 * m);
using Degree = decltype(1 * deg);
