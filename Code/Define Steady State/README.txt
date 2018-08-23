- unif: indicates the version corresponds to uniform vector fields of the testbed instead of the situation where there are flowers. use _unif version of files if applicable for boundary control law comparison.
- coverageTest_unif1 v.s. coverageTest (2KB):
	The former uses uniform scalar field whereas the latter uses scalar field defined by flower rows
- coverageTest_unif1 v.s. coverageTest_unif2:
	The former calculate gaussian blob distributions based on each robot's position at each time step whereas the latter one uses precalculated gaussian blobs and stamped on the robot instead of calculating the gaussian blobs again and again (so the version 2 is supposed to be faster)