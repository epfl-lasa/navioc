res = vario('res_ie.mat', 'res');
names = ["with $ \tilde E $ ", "without $ \tilde E $"];
h = 0.05;
t = (1:96)*h;
figure
hold on
addpath Visualization
plotmeanquart(t, res{1}.D_train, 'k', 'NavIRL - training set')
plotmeanquart(t, res{1}.D_test, 'r', 'NavIRL - test set')
plotmeanquart(t, evaldistcv(res{1}.samples_train, h, 0), [0.2,0.5,0.0], 'ConstVel - training set');
plotmeanquart(t, evaldistcv(res{1}.samples_test, h, 0), 'c', 'ConstVel - test set');
l = legend();
l.Interpreter = 'Latex';
xlabel('$ t $ [s]', 'Interpreter', 'Latex')
ylabel('error [m]', 'Interpreter', 'Latex')