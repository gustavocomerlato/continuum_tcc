import sys
import argparse
import re
import xml.etree.ElementTree as et
import math
import os
import createContinuumSegment as sg
import numpy as np

parser = argparse.ArgumentParser(description='Utilitário para gerar URDF de um robô pseudo-flexível.')
# Required positional argument
parser.add_argument('nS', type=int,
					help='Número de segmentos do robô. Número de módulos diferentes.')
parser.add_argument('nJ', type=str,
					help='Número de juntas em cada segmento. Exemplo: \'[1 2 3]\'')
parser.add_argument('len', type=str,
					help='Comprimento dos elos em cada segmento. Equivalente a "b" no modelo. Exemplo: \'[0.2 0.3 1.0]\'')
parser.add_argument('alpha', type=str,
					help='Rotação de uma seção do robô com relação à anterior. Exemplo: \'[0.2 0.3 1.0]\'')
parser.add_argument('rad', type=str,
					help='Raio dos discos/vértebras em cada segmento. Equivalente a "a" no modelo. Exemplo: \'[0.1 0.2 0.1]\'')
parser.add_argument('--vis', type=bool, default=True, action=argparse.BooleanOptionalAction,
					help='Especifica se juntas devem mimicar umas as outras na mesma seção. Default: Falso')
args = parser.parse_args()
#interpretando os argumentos
nSegs=args.nS
nJoints_str=args.nJ
linkLen_str=args.len
vertRad_str=args.rad
alpha_str=args.alpha
visible=args.vis

re_int = re.compile('\d+')
re_float = re.compile('[-+]?(?:\d*\.\d+|\d+)') #expressão regex para obter um float

nJoints=list(map(int,re.findall(re_int,nJoints_str)))
linkLen=list(map(float,re.findall(re_float,linkLen_str)))
vertRad=list(map(float,re.findall(re_float,vertRad_str)))
alpha=list(map(float,re.findall(re_float,alpha_str)))
alpha = [x*math.pi/180 for x in alpha]
#validando as listas
nJoints=sg.list_val(nJoints,nSegs)
linkLen=sg.list_val(linkLen,nSegs)
vertRad=sg.list_val(vertRad,nSegs)

robLen=[linkLen[x] * nJoints[x] for x in range(0,len(linkLen))]
print('Robô com seções de comprimento '+str(robLen)+' e '+str(nJoints)+' juntas criado!')
cylRadius=[x / 10.0 for x in vertRad]
vertLen=[x / 10.0 for x in vertRad]
lim=30*math.pi/180

segfn=list(map(''.join, zip(["Section"]*nSegs, map(str,list(range(nSegs)))))) #gera uma lista tipo "Section0","Section1", etc.

packagePath = os.path.dirname(os.getcwd())
urdfpath = packagePath+"/urdf/"

with open(urdfpath+"continuumRobot.urdf.xacro",'w') as file:
	file.write("<?xml version=\"1.0\"?>\n")
	file.write("<robot name=\"continuum_robot\" xmlns:xacro=\"http://www.ros.org/wiki/xacro\">\n")
	file.write("\n")
	file.write("    <xacro:property name=\"M_PI\" value=\"3.1415926535897931\"/>\n")
	file.write("\n")

	for segment in range(0,nSegs):
		file.write("<xacro:include filename=\"$(find continuum_description)/urdf/Section"+str(segment)+".urdf.xacro\" />\n")

	file.write("<link name=\"world\"/>\n")

	file.write("<joint name=\"fixed\" type=\"fixed\">\n")
	file.write("	<parent link=\"world\"/>\n")
	file.write("	<child link=\"originLink\"/>\n")
	# file.write("		<origin xyz=\"0.0 0.0 "+str((sum(robLen)+0.2))+"\" rpy=\"${M_PI} 0.0 0.0\"/>\n")
	file.write("</joint>\n")
	file.write("\n")
	file.write("<link name=\"originLink\"/>\n")
	file.write("\n")
	file.write("<joint name=\"baseJoint\" type=\"fixed\">\n")
	file.write("	<parent link=\"originLink\"/>\n")
	file.write("	<child link=\"S0B\" />\n")
	file.write("	<origin xyz=\"0.0 0.0 0.0\" rpy=\"0.0 0.0 "+str(alpha[0])+"\" />\n")
	file.write("</joint>\n")
	file.write("\n")


	for segment in range(0,nSegs):
		file.write("<xacro:Section"+str(segment)+" name=\"Section"+str(segment)+"\"/>\n")
		file.write("\n")
		print(nSegs)
		if (segment<nSegs-1):
			file.write("<joint name=\"JS"+str(segment)+"S"+str(segment+1)+"\" type=\"fixed\">\n")
			file.write("	<parent link=\"S"+str(segment)+"E\"/>\n")
			file.write("	<child link=\"S"+str(segment+1)+"B\" />\n")
			file.write("	<origin xyz=\"0.0 0.0 0.0\" rpy=\"0.0 0.0 "+str(alpha[segment+1])+"\" />\n")
			file.write("</joint>\n")
			file.write("\n")

	file.write("\n")
	file.write("<joint name=\"endJoint\" type=\"fixed\">\n")
	file.write("	<parent link=\"S"+str(nSegs-1)+"E\"/>\n")
	file.write("	<child link=\"endLink\" />\n")
	file.write("	<origin xyz=\"0.0 0.0 0.0\" rpy=\"0.0 0.0 0.0\" />\n")
	file.write("</joint>\n")
	file.write("\n")

	file.write("<link name=\"endLink\">\n")
	# file.write("	<collision>")
	# file.write("		<origin xyz=\"0.0 0.0 0.0\" rpy=\"0 "+str(0/2)+" 0\"/>\n")
	# file.write("		<geometry>\n")
	# file.write("			<cylinder length=\""+str(vertLen[-1])+"\" radius=\""+str(vertRad[-1])+"\"/>\n")
	# file.write("		</geometry>\n")
	# file.write("	</collision>\n")
	# if(visible):
	# 	file.write("\n")
	# 	file.write("	<visual>\n")
	# 	file.write("		<origin xyz=\"0.0 0.0 0.0\" rpy=\"0 "+str(0/2)+" 0\"/>\n")
	# 	file.write("		<geometry>\n")
	# 	file.write("			<cylinder length=\""+str(vertLen[-1])+"\" radius=\""+str(vertRad[-1])+"\"/>\n")
	# 	file.write("		</geometry>\n")
	# 	file.write("		<material name=\"Transparent\"/>\n")
	# 	file.write("	</visual>\n")
	file.write("</link>\n")

	file.write("</robot>\n")
file.close()
for segment in range(0,nSegs):
	sg.write_segment(urdfpath,segment,nJoints[segment],vertLen[segment],vertRad[segment],linkLen[segment],cylRadius[segment],lim)
	

##Escreve os arquivos XML de launch associado
contPath = os.path.dirname(packagePath)
cmPath = contPath+"/continuum_manipulator/launch/"
launchPath = packagePath+"/launch/"


with open(cmPath+"continuum_sections.launch.xml",'w') as file:
	file.write("<launch>\n")
	#TODO: alterar isso para ser o include do continuum_robot
	# file.write("<include file=\"$(find-pkg-share continuum)/launch/display_xacro.launch.xml\">\n")
	# file.write("    <arg name=\"use_sim_time\" value=\"false\"/>\n")
	# file.write("    <arg name=\"use_gui\" value=\"false\"/>\n")
	# file.write("</include>\n")
	for ss in range (1,nSegs+1):
		nm="continuum_section_"+str(ss)
		file.write("<node name=\""+nm+"\" pkg=\"continuum_manipulator\" exec=\"continuum_section\"\n")
		file.write("	args=\"-p $(find-pkg-share continuum_description)/config/"+nm+".yaml "+nm+"\">\n")
		file.write("	<param from=\"$(find-pkg-share continuum_description)/config/"+nm+".yaml\"/>\n")
		file.write("</node>\n")
	file.write("</launch>\n")

##Escreve os arquivos YAML de configuração
with open(packagePath+"/config/continuum.yaml",'w') as file:
	file.write("cable_untangler:\n")
	file.write("  ros__parameters:\n")
	file.write("    num_sec: "+str(nSegs)+"\n")
	# file.write("    num_joints_urdf: "+str(nJoints.sum()+2*nSegs)+"\n")
	file.write("\n")
	file.write("continuum_robot:\n")
	file.write("  ros__parameters:\n")
	file.write("    num_sec: "+str(nSegs)+"\n")
	file.write("    alpha: "+ str(np.cumsum(alpha).tolist())+"\n")
	file.write("    num_joints: "+str(nJoints)+"\n")
	file.write("    b: "+str([x / 2 for x in linkLen])+"\n")
	
#Para cada seção
for ss in range (1,nSegs+1):
	with open(packagePath+"/config/continuum_section_"+str(ss)+".yaml",'w') as file:
		file.write("continuum_section_"+str(ss)+":\n")
		file.write("  ros__parameters:\n")
		file.write("    num_sec: "+str(ss)+"\n")
		file.write("    num_joints: "+str(nJoints[ss-1])+"\n")
		file.write("    num_cables: "+str(nSegs*4)+"\n")
		file.write("    num_sec_cables: "+str((nSegs+1-ss)*4)+"\n")
		file.write("    num_joints_urdf: "+str(nJoints[ss-1]+2)+"\n")
		file.write("    a: "+str(vertRad[ss-1:])+"\n")
		file.write("    b: "+str(linkLen[ss-1]/2)+"\n")
		file.write("    alpha: "+str(np.cumsum(alpha[ss-1:]).tolist())+"\n")
		file.write("    preload: "+str(0.0)+"\n")


