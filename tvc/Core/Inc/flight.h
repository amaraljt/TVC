#ifndef FLIGHT_H
#define FLIGHT_H

typedef enum {
    FLIGHT_DISARM = 0,
    FLIGHT_ARMED,
    FLIGHT_BOOST,
    FLIGHT_COAST,
    FLIGHT_LANDED
} FlightState;

extern volatile FlightState g_flight_state;

void Flight_Update(void);

#endif /* FLIGHT_H */
