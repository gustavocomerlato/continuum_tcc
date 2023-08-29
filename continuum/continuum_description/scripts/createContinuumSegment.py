import sys
import argparse
import re
import xml.etree.ElementTree as et
import math
import os
m_pi=math.pi

rho=7.8e3
cyl_radius=0.00001

#algumas definições de função
def insert_disk(file,jntname,parentlink,diskname,phijnt,r=0,p=0,y=0,x_=0,y_=0,z_=0):
    file.write("\n")
    file.write("        <joint name=\""+jntname+"\" type=\"fixed\">\n")
    # file.write("    	    <mimic joint=\""+phijnt+"\" multiplier=\"-1\" />\n")
    file.write("    	    <axis xyz=\"0 0 1\"/>\n")
    file.write("    	    <parent link=\""+parentlink+"\"/>\n")
    file.write("    	    <child link=\""+diskname+"\" />\n")
    file.write("        	<origin rpy=\""+str(r)+" "+str(p)+" "+str(y)+"\" xyz=\""+str(x_)+" "+str(y_)+" "+str(z_)+"\"/>\n")
    # file.write("            <dynamics damping=\"0.2\"/>\n")
    file.write("        </joint>\n")
    file.write("\n")

    file.write("        <link name=\""+diskname+"\">\n")
    # file.write("            <inertial>\n")
    # file.write("                <mass value=\" ${disk_mass}\"/>\n")
    # file.write("                <origin xyz=\"0 0 0\"  rpy=\"0 0 0\"/>\n")
    # file.write("    	        <inertia ixx=\"${disk_Ixx}\" ixy=\"0.0\" ixz=\"0.0\" iyy=\"${disk_Iyy}\" iyz=\"0.0\" izz=\"${disk_Izz}\"/>\n")
    # file.write("            </inertial>\n")
    # file.write("\n")
    # file.write("            <collision>\n")
    # file.write("                <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n")
    # file.write("    		    <geometry>\n")
    # file.write("    			    <cylinder length=\"${vert_len}\" radius=\"${vert_rad}\"/>\n")
    # file.write("    		    </geometry>\n")
    # file.write("            </collision>\n")
    # file.write("\n")
    file.write("    	    <visual>\n")
    file.write("    		    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n")
    file.write("    		    <geometry>\n")
    file.write("    			    <cylinder length=\"${vert_len}\" radius=\"${vert_rad}\"/>\n")
    file.write("    		    </geometry>\n")
    file.write("    		    <material name=\"Red\">\n")
    file.write("    		    	<color rgba=\"1 0 0 1\"/>\n")
    file.write("    	    	</material>\n")
    file.write("        	</visual>\n")
    file.write("        </link>\n")
    file.write("\n")

def insert_link(file,link_name,link_len,r=0,p=0,y=0,x_=0,y_=0,z_=0):
    global rho
    global cyl_radius
    lm = m_pi*link_len*cyl_radius*cyl_radius*rho
    l_ixx = l_iyy = lm/12 * (3*cyl_radius**2 + link_len**2)
    l_izz = lm/2 * cyl_radius**2
    file.write("\n")
    file.write("        <link name=\""+link_name+"\">\n")
    # file.write("            <inertial>\n")
    # file.write("                <mass value=\""+str(lm)+"\"/>\n")
    # file.write("                <origin xyz=\"0 0 0\"  rpy=\"0 0 0\"/>\n")
    # file.write("    	        <inertia ixx=\""+str(l_ixx)+"\" ixy=\"0.0\" ixz=\"0.0\" iyy=\""+str(l_iyy)+"\" iyz=\"0.0\" izz=\""+str(l_izz)+"\"/>\n")
    # file.write("            </inertial>\n")
    # file.write("\n")
    # file.write("            <collision>\n")
    # file.write("                <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n")
    # file.write("    		    <geometry>\n")
    # file.write("    			    <cylinder length=\""+str(link_len)+"\" radius=\"${cyl_radius}\"/>\n")
    # file.write("    		    </geometry>\n")
    # file.write("            </collision>\n")
    # file.write("\n")
    file.write("    	    <visual>\n")
    file.write("    		    <origin rpy=\""+str(r)+" "+str(p)+" "+str(y)+"\" xyz=\""+str(x_)+" "+str(y_)+" "+str(z_)+"\"/>\n")
    file.write("    		    <geometry>\n")
    file.write("    			    <cylinder length=\""+str(link_len)+"\" radius=\"${cyl_radius}\"/>\n")
    file.write("    		    </geometry>\n")
    file.write("    		    <material name=\"Black\">\n")
    file.write("    		    	<color rgba=\"1 1 1 1\"/>\n")
    file.write("    	    	</material>\n")
    file.write("        	</visual>\n")
    file.write("        </link>\n")

def insert_joint(file,joint_name,parent,child,limit,mimic=False,mimiced_joint="Undefined",r=0,p=0,y=0,x_=0,y_=0,z_=0):
    file.write("\n")
    file.write("        <joint name=\""+joint_name+"\" type=\"revolute\">\n")
    file.write("    	    <parent link=\""+parent+"\"/>\n")
    file.write("    	    <child link=\""+child+"\"/>\n")
    file.write("    	    <origin rpy=\""+str(r)+" "+str(p)+" "+str(y)+"\" xyz=\""+str(x_)+" "+str(y_)+" "+str(z_)+"\"/>\n")
    file.write("            <limit lower=\""+str(-limit)+"\" upper=\""+str(limit)+"\" velocity=\"100.0\" effort=\"100.0\"/>\n")
    if(mimic):
        file.write("    	    <mimic joint=\""+mimiced_joint+"\" />\n")
    file.write("    	    <axis xyz=\"0 1 0\"/>\n")
    # file.write("            <dynamics damping=\"0.2\"/>\n")
    file.write("        </joint>\n")
    file.write("\n")

    # file.write("        <gazebo reference=\""+joint_name+"\">\n")
    # # file.write("            <springStiffness>1.0</springStiffness>\n")
    # # file.write("            <springReference>0.0</springReference>\n")
    # # file.write("            <implicitSpringDamper>1.0</implicitSpringDamper>\n")
    # file.write("        </gazebo>\n")
    file.write("\n")


def list_val(list_args,num):
	if(len(list_args)==num):
		return list_args
	elif(len(list_args)!=1):
		print('Comprimento da lista não bate com o comprimento esperado!')
		print('Preenchendo com o primeiro valor...')
	return [list_args[0]] * num


def write_segment(urdfpath,segment,n_disks,vert_len,vert_rad,link_len,cyl_radius_,jointlimit,rho_=78000,visible=True):
    with open(urdfpath+"Section"+str(segment)+'.urdf.xacro','w') as file:
        disk=0
        global rho
        rho = rho_
        global cyl_radius
        cyl_radius = cyl_radius_
        file.write("<?xml version=\"1.0\"?>\n")
        file.write("\n")
        file.write("<robot name=\"continuum_robot\" xmlns:xacro=\"http://www.ros.org/wiki/xacro\">\n")
        file.write("\n")
        file.write("    <xacro:macro name=\"Section"+str(segment)+"\" params=\"name\">\n")
        file.write("\n")
        file.write("    <xacro:property name=\"vert_len\" value=\""+str(vert_len)+"\" />\n")
        file.write("    <xacro:property name=\"vert_rad\" value=\""+str(vert_rad)+"\" />\n")
        file.write("    <xacro:property name=\"link_len\" value=\""+str(link_len)+"\" />\n")
        file.write("    <xacro:property name=\"cyl_radius\" value=\""+str(cyl_radius)+"\" />\n")
        #calculo de inercia
        
        lm = m_pi*link_len*cyl_radius*cyl_radius*rho
        l_ixx = l_iyy = lm/12 * (3*cyl_radius**2 + link_len**2)
        l_izz = lm/2 * cyl_radius**2
        dm = m_pi*vert_len*vert_rad*vert_rad*rho
        d_ixx = d_iyy = dm/12 * (3*vert_rad**2 + vert_len**2)
        d_izz = dm/2 * vert_rad**2
        file.write("    <xacro:property name=\"link_mass\" value=\""+str(lm)+"\" />\n")
        file.write("    <xacro:property name=\"link_Ixx\" value=\""+str(l_ixx)+"\" />\n")
        file.write("    <xacro:property name=\"link_Iyy\" value=\""+str(l_iyy)+"\" />\n")
        file.write("    <xacro:property name=\"link_Izz\" value=\""+str(l_izz)+"\" />\n")
        file.write("    <xacro:property name=\"disk_mass\" value=\""+str(dm)+"\" />\n")
        file.write("    <xacro:property name=\"disk_Ixx\" value=\""+str(d_ixx)+"\" />\n")
        file.write("    <xacro:property name=\"disk_Iyy\" value=\""+str(d_iyy)+"\" />\n")
        file.write("    <xacro:property name=\"disk_Izz\" value=\""+str(d_izz)+"\" />\n")
        
        file.write("\n")
        file.write("        <link name=\"S"+str(segment)+"B\">\n")
        file.write("\n")
        # file.write("            <inertial>\n")
        # file.write("                <mass value=\"1e-5\"/>\n")
        # file.write("                <origin xyz=\"0 0 0\"  rpy=\"0 0 0\"/>\n")
        # file.write("    	        <inertia ixx=\"1e-5\" ixy=\"0.0\" ixz=\"0.0\" iyy=\"1e-5\" iyz=\"0.0\" izz=\"1e-5\"/>\n")
        # file.write("            </inertial>\n")
        # file.write("\n")
        # file.write("            <collision>\n")
        # file.write("                <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n")
        # file.write("                <geometry>\n")
        # file.write("    			    <cylinder length=\"0.00001\" radius=\"0.00001\"/>\n")
        # file.write("    		    </geometry>\n")
        # file.write("            </collision>\n")
        # file.write("\n")
        file.write("    	    <visual>\n")
        file.write("    		    <origin rpy=\"0 0 0\" xyz=\"0 0 0 \"/>\n")
        file.write("    		    <geometry>\n")
        file.write("    			    <cylinder length=\"${vert_len}\" radius=\"${vert_rad}\"/>\n")
        file.write("    		    </geometry>\n")
        file.write("    	    	<material name=\"Transparent\">\n")
        file.write("    		    	<color rgba=\"0.5 0.5 0.5 0.5\"/>\n")
        file.write("    	    	</material>\n")
        file.write("        	</visual>\n")
        file.write("\n")
        file.write("        </link>\n")
        file.write("\n")

        file.write("        <joint name=\"S"+str(segment)+"JT_in\" type=\"continuous\">\n")
        file.write("    	    <parent link=\"S"+str(segment)+"B\"/>\n")
        file.write("    	    <child link=\"S"+str(segment)+"L0\" />\n")
        file.write("    	    <origin xyz=\"0.0 0.0 0.0\" rpy=\"0.0 0.0 0.0\" />\n")
        file.write("            <axis xyz=\"0 0 1\"/>\n")
        # file.write("            <dynamics damping=\"0.2\"/>\n")
        file.write("        </joint>\n")
        file.write("\n")

        insert_link(file,"S"+str(segment)+"L0",link_len/2,z_=link_len/4)


        djname="S"+str(segment)+"D"+str(disk)+"J"
        dname="S"+str(segment)+"D"+str(disk)
        insert_disk(file,djname,"S"+str(segment)+"L0",dname,"S"+str(segment)+"JT_in",z_=0)

        for disk in range(0,n_disks):
            link_name="S"+str(segment)+"L"+str(disk+1)
            jname="S"+str(segment)+"J"+str(disk)
            jname_1="S"+str(segment)+"J"+str(disk-1)
            djname="S"+str(segment)+"D"+str(disk+1)+"J"
            dname="S"+str(segment)+"D"+str(disk+1)

            # if (disk!=0):
            parent_name="S"+str(segment)+"L"+str(disk)
            # else:
            #     parent_name="S"+str(segment)+"B"

            ##Junta de rotação número 1
            if (disk==0):
                insert_joint(file,jname,parent_name,link_name,limit=jointlimit,mimic=False,mimiced_joint=jname_1,z_=link_len/2)
            else:
                insert_joint(file,jname,parent_name,link_name,limit=jointlimit,mimic=True,mimiced_joint=jname_1,z_=link_len)
                # insert_joint(file,jname,parent_name,link_name,True,jname_1,p=-m_pi/2,y=m_pi/2,x_=link_len)
            file.write("\n")

            ##Elos entre juntas
            if(disk<n_disks-1):
                # insert_link(file,link_name,link_len/2,visible,r=m_pi/2,y=m_pi/2,z_=link_len/2)
                insert_link(file,link_name,link_len,z_=link_len/2)
            else:
                insert_link(file,link_name,link_len/2,z_=link_len/4)
            file.write("\n")

            # ##Discos
            if(disk<n_disks-1):
                insert_disk(file,djname,link_name,dname,"S"+str(segment)+"JT_in",z_=link_len/2)
        file.write("\n")
        file.write("        <joint name=\"S"+str(segment)+"JT_out\" type=\"continuous\">\n")
        file.write("    	    <parent link=\""+link_name+"\"/>\n")
        file.write("    	    <child link=\"S"+str(segment)+"E\" />\n")
        file.write("    	    <origin rpy=\"0.0 0.0 0.0\" xyz=\"0.0 0.0 ${link_len/2}\" />\n")
        file.write("    	    <axis xyz=\"0 0 1\"/>\n")
        file.write("    	    <mimic joint=\"S"+str(segment)+"JT_in\" multiplier=\"-1\" />\n")
        # file.write("            <dynamics damping=\"0.2\"/>\n")
        file.write("        </joint>\n")
        file.write("\n")
        file.write("\n")
        file.write("        <link name=\"S"+str(segment)+"E\">\n")
        # file.write("            <inertial>\n")
        # file.write("                <mass value=\" ${disk_mass}\"/>\n")
        # file.write("                <origin xyz=\"0 0 0\"  rpy=\"0 0 0\"/>\n")
        # file.write("    	        <inertia ixx=\"${disk_Ixx}\" ixy=\"0.0\" ixz=\"0.0\" iyy=\"${disk_Iyy}\" iyz=\"0.0\" izz=\"${disk_Izz}\"/>\n")
        # file.write("            </inertial>\n")
        # file.write("\n")
        # file.write("            <collision>\n")
        # file.write("                <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n")
        # file.write("    		    <geometry>\n")
        # file.write("    			    <cylinder length=\"${vert_len}\" radius=\"${vert_rad}\"/>\n")
        # file.write("    		    </geometry>\n")
        # file.write("            </collision>\n")
        file.write("\n")
        file.write("    	    <visual>\n")
        file.write("    		    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n")
        file.write("    		    <geometry>\n")
        file.write("    			    <cylinder length=\"${vert_len}\" radius=\"${vert_rad}\"/>\n")
        file.write("    		    </geometry>\n")
        file.write("    		    <material name=\"Red\">\n")
        file.write("    		    	<color rgba=\"1 0 0 1\"/>\n")
        file.write("    	    	</material>\n")
        file.write("        	</visual>\n")
        file.write("        </link>\n")
        file.write("\n")
        file.write("\n")
        file.write("    </xacro:macro>\n")
        file.write("\n")
        file.write("</robot>\n")
    file.close()

