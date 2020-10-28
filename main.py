from Experimenter import Experimenter
import time

experiment_length = 6.0
experimenter = Experimenter(50)

experimenter.run_vert_experiment(experiment_length)
print "Finished vertical experiment"
time.sleep(1)
experimenter.run_phi_experiment(experiment_length)
print "Finished phi experiment"
time.sleep(1)
experimenter.run_theta_experiment(experiment_length)
print "Finished theta experiment"
time.sleep(1)
experimenter.run_psi_dot_experiment(experiment_length)
print "Finished yaw rate experiment"
time.sleep(1)
experimenter.run_complete_experiment(experiment_length)
print "Finished all experiments"
