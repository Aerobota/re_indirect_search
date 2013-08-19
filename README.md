# re_indirect_search

Indirect Object Search Algorithm for RoboEarth final demonstrator.

## Installation 

Before rosmake you may nedd to install the python module scikit-learn

	sudo apt-get install build-essential python-dev python-numpy python-setuptools python-scipy libatlas-dev libatlas3-base
	pip install -U scikit-learn 

## Usage

### Batch Learning
due to pickle issues this has to be done everytime 
-	Create a backup of the GMM model

		rosrun re_indirect_serach learn.py clean 
-	Learn all GMM models (and go for a coffee)

		rosrun re_indirect_search learn.py
-	Get a dump of all the GMM models

		rosrun re_indirect_search dump <filename>

### Inference 
-	Run the indirect_search node 

		rosrun re_indirect_search re_indirect_search
-	Test by running

		cd tests/
		./sample-inference.py
	This script uses the InferenceQuery.srv defined in this package. See script for more details.

### One Shot Learning 
-	Run the indirect_search node 

		rosrun re_indirect_search re_indirect_search
-	Test by running

		cd tests/
		./single-sample-learning.py
	This script uses the LearnQuery.srv defined in this package. See script for more details.

### Uploading to RoboEarth DB
-	The following command publishes the SemMap.msg topics to be uploaded to the RoboEarth (~1449 SemMaps)
	
		rosrun rosrun re_indirect_search uploader.py option: time[s] to sleep in between 
	





	
 


 

