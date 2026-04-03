#pragma once

#include <RF24.h>
#include <nRF24L01.h>
#include <RemoteData.h>


namespace robot::remote {
    bool connect();
    bool fetch(RemoteData& data);
}