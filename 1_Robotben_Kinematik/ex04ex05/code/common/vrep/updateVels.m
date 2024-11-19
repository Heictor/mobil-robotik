function ret=updateVels(vrep,clientID,dq,q)
vels=vrep.simxPackFloats([dq' q']);
[err]=vrep.simxSetStringSignal(clientID,'vels',vels, vrep.simx_opmode_oneshot_wait);
vrep.simxSynchronousTrigger(clientID);
end