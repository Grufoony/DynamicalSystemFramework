from dsf_cpp import mobility

sim = mobility.TrafficSimulator()
sim.setName("auto_charge_py")
# User should import a valid network first
# sim.importRoadNetwork('examples/data/edges.csv')
sim.setTimeFrame(0, 600)
# save every 60 steps: avg_stats etc.
sim.saveData(60, True, True, True, True)
# Run AutoCharge via bindings
sim.runAutoCharge(
    5,  # baseAgentCount
    30,  # dtAgent
    60,  # saveIntervalSeconds
    None,  # maxSteps
    None,  # stopMeanDensityVpk
    120,  # stabilityHoldSeconds
    120,  # stabilityCooldownSeconds
    2,  # chargeIncrement
    True,  # injectOnCharge
    "auto_charge_events.csv",  # stabilityLogFile
)
