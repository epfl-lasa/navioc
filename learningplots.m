resL2 = load('/media/david/LaCieG/data-navioc/res_learnexp_acc2sum.mat');
resL1 = load('/media/david/LaCieG/data-navioc/res_learnexp_smabsacc.mat');
h = 0.05;

addpath Visualization

t = h*(1:96)';
figure
subplot(2, 1, 1)
hold on
plotquartiles(t, resL2.D_train, [1, 0.65, 0.65])
plotquartiles(t, resL1.D_train, [0.5, 1, 1])
plot(t, mean(resL2.D_train, 2), 'r', 'DisplayName', 'NavIOC-$L^2$') %, 'LineWidth', 2)
plot(t, mean(resL1.D_train, 2), 'Color', [0,0.7,0.7], 'DisplayName', 'NavIOC-$L^1$')
plot(t, mean(resL2.D_train_cv, 2), 'b-.', 'DisplayName', 'constant velocity')
xlabel('$\Delta t$ [s]', 'Interpreter', 'Latex')
ylabel('Displacement Error [m]', 'Interpreter', 'Latex')
leg = legend();
set(leg, 'Interpreter', 'Latex');
title('Accuracy on training dataset')

subplot(2, 1, 2)
hold on
plotquartiles(t, resL2.D_test, [1, 0.65, 0.65])
plotquartiles(t, resL1.D_test, [0.5, 1, 1])
plot(t, mean(resL2.D_test, 2), 'r', 'DisplayName', 'NavIOC-$L^2$') %, 'LineWidth', 2)
plot(t, mean(resL1.D_test, 2), 'Color', [0,0.7,0.7], 'DisplayName', 'NavIOC-$L^1$')
plot(t, mean(resL2.D_test_cv, 2), 'b-.', 'DisplayName', 'constant velocity')
xlabel('$\Delta t$ [s]', 'Interpreter', 'Latex')
ylabel('Displacement Error [m]', 'Interpreter', 'Latex')
leg = legend();
set(leg, 'Interpreter', 'Latex');
title('Accuracy on test dataset')
% plotmeanquart(t, resL2.D_train, 'k', 'NavIOC-L2 - training set $ \mathcal E_1 $')
% plotmeanquart(t, resL2.D_test, 'r', 'NavIOC-L2 - test set $ \mathcal E_2 $')
% plotmeanquart(t, resL1.D_train, 'g', 'NavIOC-L1 - training set $ \mathcal E_1 $')
% plotmeanquart(t, resL1.D_test, 'y', 'NavIOC-L1 - test set $ \mathcal E_2 $')
% plotmeanquart(t, resL2.D_train_cv, [0.25, 0, 0.75], 'CV - $ \mathcal E_1 $')
% plotmeanquart(t, resL2.D_test_cv, 'c', 'CV - $ \mathcal E_2 $')


t_eth = h*(8:8:96)';
t_kretzschmar = [0, 1, 2, 3, 4]';
err_kretzschmar = [
	0
	1.6853	
	3.4051
	4.2995
	4.48
]/6.1833;
% figure
% hold on
% plotmeanquart(t_eth, resL2.D_eth, 'r', 'NavIOC-L2')
% plotmeanquart(t_eth, resL1.D_eth, 'y', 'NavIOC-L1')
% plotmeanquart(t_eth, resL2.D_eth_cv, 'c', 'CV')
% plot(t_kretzschmar, err_kretzschmar, 'ko-', 'DisplayName', 'Kretzschmar et al. (2016)')
% xlabel('$\Delta t$ [s]', 'Interpreter', 'Latex')
% ylabel('Error [m]', 'Interpreter', 'Latex')
% legend()
% title('ETH (test) set')

figure
hold on
plotquartiles(t_eth, resL2.D_eth, [1, 0.65, 0.65])
plotquartiles(t_eth, resL1.D_eth, [0.5, 1, 1])
plot(t_eth, mean(resL2.D_eth, 2), 'r', 'DisplayName', 'NavIOC-$L^2$') %, 'LineWidth', 2)
plot(t_eth, mean(resL1.D_eth, 2), 'Color', [0,0.7,0.7], 'DisplayName', 'NavIOC-$L^1$')
plot(t_eth, mean(resL2.D_eth_cv, 2), 'b-.', 'DisplayName', 'constant velocity')
plot(t_kretzschmar, err_kretzschmar, 'ko--', 'DisplayName', 'Kretzschmar et al. (2016)')
leg = legend();
set(leg, 'Interpreter', 'Latex');
xlabel('$\Delta t$ [s]', 'Interpreter', 'Latex')
ylabel('Displacement Error [m]', 'Interpreter', 'Latex')
title('Prediction on ETH (test) dataset')