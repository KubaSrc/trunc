close all; clc; clear all;

addpath('./util/NatNet_SDK_4.1/NatNetSDK/Samples/Matlab');
addpath('./util/')

nnc = connect_to_natnet();

bodies = nnc.getFrame().RigidBodies

1000.*(bodies(2).y - bodies(1).y)