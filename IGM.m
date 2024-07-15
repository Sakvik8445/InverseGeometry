robot = rigidBodyTree('DataFormat','column','MaxNumBodies',3);
L1 = 0.3;
L2 = 0.3;
L3 = 0.3;
body = rigidBody('link1');
joint = rigidBodyJoint('joint1','revolute');
setFixedTransform(joint,trvec2tform([0 0 0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot,body,'base')
body = rigidBody('link2');
joint = rigidBodyJoint('joint2','revolute');
setFixedTransform(joint,trvec2tform([L1 0 0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot,body,'link1');

body = rigidBody('link3');
joint = rigidBodyJoint('joint3','revolute');
setFixedTransform(joint,trvec2tform([L2 0 0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot,body,'link2')

body = rigidBody('tool');
joint = rigidBodyJoint('fix1','revolute');
setFixedTransform(joint,trvec2tform([L2 0 0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot,body,'link3');

showdetails(robot)
t = (0:0.2:10)';
count = length(t);
center = [0.3 0.1 0];
radius = 0.15;
theta = t*(2*pi/t(end));
points = center + radius*[cos(theta) sin(theta) zeros(size(theta))];
q0 = homeConfiguration(robot);
ndof = length(q0);
qs = zeros(count,ndof);
ik = inverseKinematics("RigidBodyTree",robot);
weights = [0,0,0,1,1,0];
endEffector = 'tool';
qinitial = q0;
for i = 1:count
    point = points(i,:);
    qsol = ik(endEffector,trvec2tform(point),weights,qinitial);
    qs(i,:) = qsol;
    qinitial = qsol;
end

figure
show(robot,qs(1,:)');
view(2)
ax = gca;
ax.Projection = 'perspective';
hold on;
plot(points(:,1),points(:,2),'k');
axis([-0.1 0.7 -0.3 0.5])
framesPerSecond = 15;
r = rateControl(framesPerSecond);
for i = 1:count
    show(robot,qs(i,:)','PreservePlot',false);
    drawnow
    waitfor(r);
end








