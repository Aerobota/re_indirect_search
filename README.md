# re_indirect_search

Indirect Object Search Algorithm for RoboEarth final demonstrator.

## Installation 

Before rosmake you may nedd to install the python module scikit-learn

	sudo apt-get install build-essential python-dev python-numpy python-setuptools python-scipy libatlas-dev libatlas3-base
	pip install -U scikit-learn 

## Usage

### Batch Learning
-	Create a backup of the GMM model

		rosrun re_indirect_serach learn.py clean 
-	Learn all GMM models
		rosrun re_indirect_search learn.py
-	Get a dump of all the GMM models
		rosrun re_indirect_search dump <filename>


### One Shot Learning 
-	Run the indirect_search node 
		rosrun re_indirect_search re_indirect_search
-	Test by running
		cd tests/
		./single-sample-learning.py

 
### Inference 
-	Run the indirect_search node 
		rosrun re_indirect_search re_indirect_search
-	Test by running
		cd tests/
		./sample-inference.py

 

