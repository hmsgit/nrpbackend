    @nrp.NeuronMonitor(nrp.brain.circuit[slice(0, 8, 1)], nrp.spike_recorder)
    def all_neurons_spike_monitor(t):
        return True
