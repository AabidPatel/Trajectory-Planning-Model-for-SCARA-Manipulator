function visualize_VR=SCARA_VR_VISUALIZE(q,model_3d)
% q is a timeseries output from simulink
vrsetpref('TransportTimeout',1)
vrsetpref('TransportBuffer',1)
if(model_3d)
SCARA = vrworld('SCARA.wrl');
open(SCARA); 
view(SCARA);
end
a1=1;
a2=1;
d1=1;
d2=.1;
d=.1;
L=.3;
s = max(size(q));
for t=1:1:s
    q1 = q(1,t)*180/pi;
    q2 = q(2,t)*180/pi;
    d3 = q(3,t);
    q4 = q(4,t)*180/pi;;
    SCARA_plot(q1,q2,q4,a1,a2,d1,d3,d2,L,d);
    if(model_3d)
    SCARA_VR_PLOT(SCARA,q1,q2,q4,-d3,d);
    end
    pause(.01);
end