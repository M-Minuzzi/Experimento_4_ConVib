clc, clear, close all;
%% Dados iniciais

a = 1;
b = 7;
c = 0;
d = 1;
e = 7;

%% Termos da função de transferência

a_1 = a*d;
b_1 = (a*e+b*d);
c_1 = (b*e+c*d);
d_1 = (c*e-1);


% Definindo a função de transferência
s = tf('s');

ft = (d*s + e) / (a_1*s^3+b_1*s^2+c_1*s+d_1);

%% Plot das raízes

sys=tf([d e], [a_1 b_1 c_1 d_1]);
figure;
rlocus(sys);
title('Mapa de Polos e Zeros - Função de Transferência');
% print('RootLocus_transfer_fcn', '-dpdf'); % Salva a figura em PDF
exportgraphics(gcf, 'RootLocus_transfer_fcn.pdf', 'ContentType', 'vector');


%% Plot do diagrama de bode
figure;
bode(sys);
title('Diagrama de Bode - Função de Transferência');
% print('Bode_transfer_fcn', '-dpdf'); % Salva a figura em PDF
exportgraphics(gcf, 'Bode_transfer_fcn.pdf', 'ContentType', 'vector');

%% Parâmetros do PID

K_cr = (a*d*(b*e+c*d))/(a*e+b*d);

P_cr = (c*e-1)/(a*d);

Kp = 0.6*K_cr;

Ti = 0.5*P_cr;

Td = 0.125*P_cr;

%% Montagem da Função de Transferência do PID

Gc = Kp * (1 + (1/(Ti*s)) + (Td*s));

%% Função transferência resultante
ft_r = Gc*sys;

%% Análise em Malha Aberta
figure;
rlocus(ft_r); 
title('Mapa de Polos e Zeros - Malha Aberta');

%% Análise em Malha Fechada (Sistema com Realimentação)
% Para ver a estabilidade REAL do sistema final, fechamos a malha:
sys_cl = feedback(ft_r, 1); % Assume realimentação unitária negativa

figure;
rlocus(sys_cl);
title('Mapa de Polos e Zeros - Malha Fechada (Estabilidade Final)');
% print('RootLocus_sistema_controlado', '-dpdf');
exportgraphics(gcf, 'RootLocus_sistema_controlado.pdf', 'ContentType', 'vector');


%%
figure;
bode(sys_cl);
title('Diagrama de Bode - Sistema Controlado');
% print('bode_sistema_controlado', '-dpdf');
exportgraphics(gcf, 'bode_sistema_controlado.pdf', 'ContentType', 'vector');

%% Simulação com simulink

simulink

%% salvar pdfs
dados = out.ScopeData; 

% Criar uma figura nova
figure('Color', 'w');
plot(dados.time, dados.signals.values, 'LineWidth', 1.5);


grid on;
xlabel('Tempo (s)');
ylabel('Amplitude');
title('Resposta ao Degrau do Sistema com PID');
set(gca, 'FontSize', 12);

% EXPORTAR PARA PDF (VETORIAL)
exportgraphics(gcf, 'resposta_degrau_pid_tune_fast.pdf', 'ContentType', 'vector');