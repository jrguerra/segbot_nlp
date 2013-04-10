#!/usr/bin/env python
# -*- coding: utf-8 -*-
#########################################################################################
#	                                _     						#
#	  __ _ ___ _ __   ___  ___  ___| |__  						#
#	 / _` / __| '_ \ / _ \/ _ \/ __| '_ \ 						#
#	| (_| \__ \ |_) |  __/  __/ (__| | | |						#
#	 \__, |___/ .__/ \___|\___|\___|_| |_|						#
#	 |___/    |_|                         						#
#											#
# ros package for speech recognition using Google Speech API				#
# run using 'rosrun gspeech gspeech.py'							#
# it creats and runs a node named gspeech						#
# the node gspeech publishes two topics- /speech and /confidence			#
# the topic /speech contains the reconized speech string				#
# the topic /confidence contains the confidence level in percentage of the recognization#
#											#
#											#
# written by achuwilson									#
# 30-06-2012 , 3.00pm									#
# achu@achuwilson.in									#
#########################################################################################
import roslib; roslib.load_manifest('segbot_nlp') 
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int8
import sys
sys.path.append("/home/nlpros/ros/rosbuild_ws/segbot/segbot_nlp/src/segbot_nlp/msg")
import VoiceCommand
import shlex,subprocess,os
cmd1='sox -r 48000 -t alsa default recording.flac silence 1 0.1 1% 1 1.5 1%'
cmd2='wget -q -U "Mozilla/5.0" --post-file recording.flac --header="Content-Type: audio/x-flac; rate=48000" -O - "http://www.google.com/speech-api/v1/recognize?lang=en-us&client=chromium"'


def speech():


	rospy.init_node('gspeech')
	pubs = rospy.Publisher('filename', String)
	pubc = rospy.Publisher('command_message', _VoiceCommand);
	args2 = shlex.split(cmd2)

	while not rospy.is_shutdown():
		nb = raw_input('Press Enter to Give a Command')
		os.system('sox -r 48000 -t alsa default recording.flac silence 1 0.1 1% 1 1.5 1%')	
		output,error = subprocess.Popen(args2,stdout = subprocess.PIPE, stderr= subprocess.PIPE).communicate()
		
		if not error and len(output)>16:
			a = eval(output)
			confidence= a['hypotheses'][0]['confidence']
			confidence= confidence*100
			data=a['hypotheses'][0]['utterance']
			print "*****************************\n"
			print "Received the Following Input\n"
			print data
			print "*****************************\n"
			if data == "stop" or data == "halt":
				x = VoiceCommand()
				VoiceCommand.commandCode = 4
				pubc.publish(x)
			else:
				filename= subprocess.Popen(["perl", "/nishome/nlpros/ros/rosbuild_ws/segbot/segbot_nlp/src/gspeech/scrape.pl", data], stdout=subprocess.PIPE).communicate()
				filename=filename[0].rstrip('\n')
				pubs.publish(String(filename))
				print String(filename), confidence
		
 

if __name__ == '__main__':
	try:
		speech()
	except rospy.ROSInterruptException: pass
	except KeyboardInterrupt:
		sys.exit(1)
   
