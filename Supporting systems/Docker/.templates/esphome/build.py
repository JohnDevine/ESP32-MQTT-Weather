#!/usr/bin/python3
# -*- coding: utf-8 -*-

issues = {} # Returned issues dict
buildHooks = {} # Options, and others hooks
haltOnErrors = True

import os
import sys

global templatesDirectory
global currentServiceName # Name of the current service
global generateRandomString

from deps.consts import templatesDirectory
from deps.common_functions import generateRandomString


# Main wrapper function. Required to make local vars work correctly
def main():

	global toRun # Switch for which function to run when executed
	global buildHooks # Where to place the options menu result
	global issues # Returned issues dict
	global haltOnErrors # Turn on to allow erroring

	# runtime vars
	portConflicts = []

	# This lets the menu know whether to put " >> Options " or not
	# This function is REQUIRED.
	def checkForOptionsHook():
		try:
			buildHooks["options"] = callable(runOptionsMenu)
		except:
			buildHooks["options"] = False
			return buildHooks
		return buildHooks

	# This function is REQUIRED.
	def checkForPreBuildHook():
		try:
			buildHooks["preBuildHook"] = callable(preBuild)
		except:
			buildHooks["preBuildHook"] = False
			return buildHooks
		return buildHooks

	# This function is REQUIRED.
	def checkForPostBuildHook():
		try:
			buildHooks["postBuildHook"] = callable(postBuild)
		except:
			buildHooks["postBuildHook"] = False
			return buildHooks
		return buildHooks

	# This function is REQUIRED.
	def checkForRunChecksHook():
		try:
			buildHooks["runChecksHook"] = callable(runChecks)
		except:
			buildHooks["runChecksHook"] = False
			return buildHooks
		return buildHooks

	# This service will not check anything unless this is set
	# This function is optional, and will run each time the menu is rendered
	def runChecks():
		checkForIssues()
		return []

	# This function is optional, and will run after the docker-compose.yml file is written to disk.
	def postBuild():
		return True

	# This function is optional, and will run just before the build docker-compose.yml code.
	def preBuild():
		return True

  # #####################################
  # Supporting functions below
  # #####################################

	def doCustomSetup() :

		import os
		import re
		import subprocess
		from os.path import exists

		def copyUdevRulesFile(templates,rules) :

			# the expected location of the rules file in the template is the absolute path ...
			SOURCE_PATH = templates + '/' + currentServiceName + '/' + rules

			# the rules file should be installed at the following absolute path...
			TARGET_PATH = '/etc/udev/rules.d' + '/' + rules

			# does the target already exist?
			if not exists(TARGET_PATH) :

				# no! does the source path exist?
				if exists(SOURCE_PATH) :

					# yes! we should copy the source to the target
					subprocess.call(['sudo', 'cp', SOURCE_PATH, TARGET_PATH])

					# sudo cp sets root ownership but not necessarily correct mode
					subprocess.call(['sudo', 'chmod', '644', TARGET_PATH])

		def setEnvironment (path, key, value) :

			# assume the variable should be written
			shouldWrite = True

			# does the target file already exist?
			if exists(path) :

				# yes! open the file so we can search it
				env_file = open(path, 'r+')

				# prepare to read by lines
				env_data = env_file.readlines()

				# we are searching for...
				expression = '^' + key + '='

				# search by line
				for line in env_data:
					if re.search(expression, line) :
						shouldWrite = False
						break

			else :
	
				# no! create the file
				env_file = open(path, 'w')
	
			# should the variable be written?
			if shouldWrite :
				print(key + '=' + value, file=env_file)

			# done with the environment file
			env_file.close()

		copyUdevRulesFile(
			os.path.realpath(templatesDirectory),
			'88-tty-iotstack-' + currentServiceName + '.rules'
		)

		# the environment file is located at ...
		DOT_ENV_PATH = os.path.realpath('.') + '/.env'

		# check/set environment variables
		setEnvironment(DOT_ENV_PATH,'ESPHOME_USERNAME',currentServiceName)
		setEnvironment(DOT_ENV_PATH,'ESPHOME_PASSWORD',generateRandomString())


	def checkForIssues():
		doCustomSetup() # done here because is called least-frequently
		return True

	if haltOnErrors:
		eval(toRun)()
	else:
		try:
			eval(toRun)()
		except:
			pass

if currentServiceName == 'esphome':
	main()
else:
	print("Error. '{}' Tried to run 'plex' config".format(currentServiceName))
