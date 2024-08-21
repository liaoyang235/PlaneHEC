a = [0.043295, 0.24554, 0.56684]
m = data2plane(a)


Rx = X(1:3,1:3)
q =  mathtrans().rotm2quat(Rx)