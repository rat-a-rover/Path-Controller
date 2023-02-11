###############################################################################
# robonity: worldwide tech consulting
# Ramon Gonzalez (ramon@robonity.com)
# November 2017, February 2018
# Edits: Samuel Chandler (samxchander@protoinnovations.com)
#		 February 2018
# File: sttr_robonity_FC1
# File: sttr_robonity_FC2 (predict values after loading file form disk
# and pickling the filter
###############################################################################

###############################################################################
#							LIBRARIES
###############################################################################
import csv #csv
from scipy import signal#median filter
import numpy as np# numpy
import pickle #save/load model to/from disk
from sklearn.metrics import accuracy_score
from sklearn import model_selection #splitting training/testing sets
from sklearn.ensemble import AdaBoostClassifier #classifier
from sklearn.metrics import confusion_matrix
#from sklearn.tree import DecisionTreeClassifier
#from sklearn.svm import SVC #Support Vector Machine

###############################################################################
#							FUNCTIONS
###############################################################################
#Moving average filter (IMU signals: accX, accZ, pitch rate)
def fun_loadFile(fileNameCSV1, fileNameCSV2, colSLIP):
	f1 = open(fileNameCSV1, 'r')
	csv_f1 = csv.reader(f1)
	rowN = 0
	mylist1 = []
	for row in csv_f1:
		if rowN > 0:
			aux = row[0].split(';')
			try:
				accX = float(aux[14])
				accZ = float(aux[13])
				PR = float(aux[16])
				MC = float(aux[10])
				mylist1.append([MC, accZ, accX, PR])
			except:	
				import IPython
				IPython.embed()
		rowN = rowN + 1
	dataset1 = np.asarray(mylist1)	# convert from list to array
	#dataset1[:, 0] = signal.medfilt(dataset1[:, 0], 5)
	#dataset1[:, 1] = signal.medfilt(dataset1[:, 1], 5)
	#dataset1[:, 2] = signal.medfilt(dataset1[:, 2], 5)
	#dataset1[:, 3] = signal.medfilt(dataset1[:, 3], 5)

	###############################################################################
	# 3. Load dataset (gps & slip)
	f2 = open(fileNameCSV2, 'r')
	csv_f2 = csv.reader(f2)
	mylist2 = []
	rowN = 0
	for row in csv_f2:
		if rowN > 1:
			aux2 = row[0].split(';')
			mylist2.append([float(aux2[colSLIP])])	# slip class
		rowN = rowN + 1
	dataset2 = np.asarray(mylist2)	# convert from list to array
	aux3 = len(dataset2) - len(dataset1)
	aux4 = len(dataset2)
	if aux3 < 0:
		for i in range(1, abs(aux3) + 1):
			dataset2 = np.concatenate((dataset2, [dataset2[aux4 - 1]]))

	###############################################################################
	# 4. Return data
	X = dataset1  # columns with sensor data
	Y = dataset2  # ground-truth (target, class)
	return X, Y

class Filters:
	def __init__(self):
		self.moving_window = 50
		self.window = 5

	def fun_movingvar(self, X, m):
		fnum = np.zeros(m) * m + 1
		fden = m
		v1 = signal.lfilter(fnum, fden, np.power(X, 2))
		v2 = signal.lfilter(fnum, fden, X)
		v = v1 - np.power(v2, 2)
		return v

	def fun_filterSensor(self, filter_select, X, online=False):
		# Filters-FEATURES
		if (filter_select):
			# MOVING AVERAGE FILTER
			window = self.moving_window
			MCF = signal.medfilt(X[:, 0], 5)
			accZf = Filters.fun_movingvar(X[:, 1], moving_window)
			accXf = Filters.fun_movingvar(X[:, 2], moving_window)
			PRf = Filters.fun_movingvar(X[:, 3], moving_window)
		else:
			window = self.window
			MCF = signal.medfilt(X[:, 0], window)
			accZf = signal.medfilt(X[:, 1], window)
			accXf = signal.medfilt(X[:, 2], window)
			PRf = signal.medfilt(X[:, 3], window)
			#sample_size = len(X)
			#MCF = np.zeros(1,sample_size)
			#for i in xrange(sample_size):
			#	 if i < self.moving_window:
			#		 MCF

		if online:
			# If online, data passed will be the buffer, we only care about the value 
			# of the middle data point
			X_ret = X[0]
			X_ret[0] = MCF[(window-1)/2]
			X_ret[1] = accZf[(window-1)/2]
			X_ret[2] = accXf[(window-1)/2]
			X_ret[3] = PRf[(window-1)/2]
			X_ret = X_ret.reshape(1,-1)
		else:
			# If not online process the whole data set 
			X_ret = X
			X_ret[:, 0] = MCF
			X_ret[:, 1] = accZf
			X_ret[:, 2] = accXf
			X_ret[:, 3] = PRf


		return X_ret

	def get_window(self, filter_select):
		return self.moving_window if filter_select else self.window		

##############################################################################
#					MODEL CLASS THAT WILL BE PICKLED						 #
##############################################################################

class Model:
	# TODO These thresholds are used to assign ground truth continuous values
	# to classes. Ideally these should be loaded from the configuration file
	# used to develop the labelled data, Y_raw. For now we will manually
	# match the thresholds to that configuration file.
	thresh_1 = .3
	thresh_2 = .6

	# The filter mode is used to select the type of filter, 1 for moving average
	# 0 for median filter
	filter_mode = 0

	def __init__(self, set_filters, set_classifier):
		self.model_filters = set_filters
		self.model_classifier = set_classifier

	def predict(self, test):
		return self.model_classifier.predict(test)

	def apply_filters(self, X_raw, online_switch):
		return self.model_filters.fun_filterSensor(self.filter_mode, X_raw, online_switch)

	def score(self, X_test, Y_raw):
		return self.model_classifier.score(X_test, Y_raw)

	def value_to_class(self, slip):
		#TODO This function should be loaded from the configuration file used
		# to develop the labelled data, Y_raw
		return (1 if slip <= self.thresh_1 else (2 if slip <= self.thresh_2 else 3)) 

	def get_buffer_window(self):
		return self.model_filters.get_window(self.filter_mode)

###############################################################################
#							MAIN CODE
###############################################################################
def main():
	###############################################################################
	# Define wheel training data names
	wheel_dict = {0:'LF', 1:'RF', 2:'RR', 3:'LR'}
	###############################################################################
	len_wheel_dict = len(wheel_dict)
	for i in xrange(len_wheel_dict):	
		###############################################################################
		# 1. Global variables
		###############################################################################
		print('Running sttr_robonity...')
		fileNameCSV1 = [wheel_dict[i] + '/log_EXP5_' + wheel_dict[i] + '.csv', 
				wheel_dict[i] + '/log_EXP6_' + wheel_dict[i] + '.csv', 
				wheel_dict[i] + '/log_EXP8_' + wheel_dict[i] + '.csv']
		fileNameCSV2 = [wheel_dict[i] + '/log_EXP5_slip_' + wheel_dict[i] + '.csv', 
				wheel_dict[i] + '/log_EXP6_slip_' + wheel_dict[i] + '.csv', 
				wheel_dict[i] + '/log_EXP8_slip_' + wheel_dict[i] + '.csv']
		dataset = [] #full dataset (dataset1 + dataset2)
		validation_size = 0.30#90% for training, 10% for testing
		seed = 7 #Pseudo-random number generator state used for random sampling
		movingFilter = 0#if we want to use the moving average filter or the median filter
		kernel_size = 7 #length used for the median filter
		X = []
		Y = []

		###############################################################################
		# 2. Load datasets
		print('Loading datasets...')
		for ds in range(len(fileNameCSV1)):
			aux1 = fileNameCSV1[ds]
			aux2 = fileNameCSV2[ds]
			auxX, auxY = fun_loadFile(aux1, aux2, 10)
			if(ds == 0):
				X = auxX
				Y = auxY
			else:
				X = np.concatenate((X, auxX))
				Y = np.concatenate((Y, auxY))

		X_raw = X
		Y_raw = Y

		###############################################################################
		# 3. Calculating features
		print('Calculating features...')
		ff = Filters()
		X = ff.fun_filterSensor(movingFilter, X)

		###############################################################################
		# 4. Split-out validation dataset
		print('Split-out validation dataset')
		X_train, X_test, Y_train, Y_test = model_selection.train_test_split(X, Y, 
											  test_size=validation_size, random_state=seed)

		###############################################################################
		# 5. Training the best ML algorithm
		print('Training the best ML algorithm...')
		classifier = AdaBoostClassifier(base_estimator=None, n_estimators=10, 
								 learning_rate=1.0, algorithm='SAMME.R', random_state=None)
		#classifier = DecisionTreeClassifier(max_depth=5)
		#classfier = SVC(kernel='rbf', gamma=0.05, C=30.0, decision_function_shape='ovr')
		classifier.fit(X_train, Y_train.ravel())

		###############################################################################
		# 6. Create object to be pickled
		print('Creating model to be pickled...')
		model = Model(ff, classifier)

		###############################################################################
		# 7. Save the model to disk
		print('Save the model to disk...')
		#filename1 = 'filters_online.sav'
		#pickle.dump(Filters, open(filename1, 'wb'))
		filename2 = 'model_online_' + wheel_dict[i] + '.sav'
		pickle.dump(model, open(filename2, 'wb'))

		###############################################################################
		# 8. Load the model from disk
		print('Load the model from disk...')
		#filename1 = 'filters_online.sav'
		#Filters = pickle.load(open(filename1, 'rb'))
		#ff = Filters()
		#X_test = ff.fun_filterSensor(0, X_raw)
		filename2 = 'model_online_' + wheel_dict[i] + '.sav'
		#classifier_online = pickle.load(open(filename2, 'rb'))
		#prediction = classifier_online.predict(X_test)

		# Load the pickled model
		model_online = pickle.load(open(filename2, 'rb'))

		# Apply a filter to raw data, this is median filter by default
		X_test = model_online.apply_filters(X_raw, False)

		# Make a prediction based on the classifier part of the model
		prediction = model_online.predict(X_test)

		# Assess two different metrics of scoring
		result2 = accuracy_score(Y_raw, prediction)
		print("----------------------------")
		print(str(wheel_dict[i]) + ' result: ' + str(result2))
		result =  model_online.score(X_test, Y_raw)
		print(str(wheel_dict[i]) + ' result: ' + str(result))
		print("----------------------------")

		cm = confusion_matrix(Y_raw, prediction)
		m = cm.astype('float') / cm.sum(axis=1)[:, np.newaxis]
		print("Confusion Matrix")
		print(m)
		#Test buffer window
		print('Fetching buffer window from loaded model: '+ 
			   str(model_online.get_buffer_window()))

		print('END.')
		# Ramon Gonzalez (ramon@robonity.com)

if __name__ == "__main__":
	main()
