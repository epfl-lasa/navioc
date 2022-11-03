res = {vario('res_ie.mat', 'res'), vario('res_no_ie.mat', 'res')};
names = ["with $ \tilde E $ ", "without $ \tilde E $"];
C = ["k", "r"; "g", "c"];
h = 0.05;
t = (1:96)*h;
figure
hold on
for i = 1:2
	D_train = [res{i}{1}.D_train, res{i}{2}.D_train];
	D_test = [res{i}{1}.D_test, res{i}{2}.D_test];
	%plot_mean_std(t, D_train, C(i, 1), names(i)+"-train")
	plot_mean_std(t, D_test, C(i, 1), names(i)+"-test")
end
D_cv = evaldistcv([res{1}{1}.samples_train, res{1}{1}.samples_test], h);
plot_mean_std(t, D_cv, 'm', "const-vel")
l = legend();
l.Interpreter = 'Latex';
xlabel('$ t $ [s]', 'Interpreter', 'Latex')
ylabel('FDE [m]', 'Interpreter', 'Latex')

function plot_mean_std(X, Y, c, s)
	Mu = mean(Y, 2);
	Sigma = std(Y, 0, 2);
	plot(X, Mu, c, 'DisplayName', s)
	plot(X, Mu + Sigma, c, 'HandleVisibility', 'off')
	plot(X, Mu - Sigma, c, 'HandleVisibility', 'off')
end