// Copyright @2018 Pony AI Inc. All rights reserved.

#include "pnc/agents/agents.h"

#include "pnc/agents/sample/sample_agent.h"
#include "pnc/agents/hqztrue/hqztrue_agent.h"
#include "pnc/agents/table/table_agent.h"

// Register sample vehicle agent to a factory with its type name "sample_agent"
//static simulation::Registrar<::sample::SampleVehicleAgent> registrar("sample_agent");
static simulation::Registrar<::hqztrue::FrogVehicleAgent> registrar("frog_agent");
//static simulation::Registrar<::table::TableVehicleAgent> registrar("table_agent");
