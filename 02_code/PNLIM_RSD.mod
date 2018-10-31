# declaração de conjuntos e parâmetros
set ob;                     # conjunto de nós
set ol within ob cross ob;  # conjunto de circuitos
set och within ob cross ob; # conjunto de chaves

param Tb{ob};               # tipo de barra 0: carga, 1: SE
param PD{ob};               # potência ativa de demanda no nó i
param QD{ob};               # potência reativa de demanda no nó i
param R{ol};                # resistência no circuito ij
param X{ol};                # reatância no circuito ij
param Z2{ol};               # impedância no circuito ij ao quadrado
param Imax{ol};             # limite máximo da magnitude de corrente nos circuitos
param ei{och};              # estado inicial das chaves
param Imaxch{och};          # limite máximo da magnitude de corrente nas chaves
param cls;                  # custo das perdas de energia no nível d
param Vnom;                 # magnitude da tensão nominal
param Vmin;                 # magnitude de tensão mínima
param Vmax;                 # magnitude de tensão máxima

# declaração das variáveis
var Vqdr{ob};               # variável que representa o quadrado de V[i]
var PS{ob};                 # potência ativa fornecida pela subestação no  nó i
var QS{ob};                 # potência reativa fornecida pela subestação no nó i
var Iqdr{ol};               # variável que representa o quadrado de I[i,j]
var P{ol};                  # fluxo de potência ativa no circuito ij
var Q{ol};                  # fluxo de potência reativa no circuito ij
var Pch{och};               # fluxo de potência ativa na chave ij
var Qch{och};               # fluxo de potência reativa na chave ij
var w{och} binary;          # estado de operação da chave ij (= 1 fechada, = 0 aberta)

# definição da função objetivo
# minimização das perdas de energia
minimize custo_perdas:
  cls * sum {(i,j) in ol}(R[i,j] * Iqdr[i,j]);

# definição das restrições
subject to balance_potencia_ativa {i in ob}:
  sum{(j,i) in ol}(P[j,i]) - sum{(i,j) in ol}(P[i,j] + R[i,j] * Iqdr[i,j]) + 
  sum{(j,i) in och}(Pch[j,i]) - sum{(i,j) in och}(Pch[i,j]) + PS[i] = PD[i];
	
subject to balance_potencia_reativa {i in ob}:
  sum{(j,i) in ol}(Q[j,i]) - sum{(i,j) in ol}(Q[i,j] + X[i,j] * Iqdr[i,j]) + 
  sum{(j,i) in och}(Qch[j,i]) - sum{(i,j) in och}(Qch[i,j]) + QS[i] = QD[i];

subject to queda_magnitude_tensao {(i,j) in ol}:
  Vqdr[i] - 2*(R[i,j]*P[i,j] + X[i,j]*Q[i,j]) - Z2[i,j]*Iqdr[i,j] - Vqdr[j] = 0;
	
subject to calculo_magnitude_corrente_original {(i,j) in ol}:
  Vqdr[j] * Iqdr[i,j] = P[i,j]^2 + Q[i,j]^2;
					   
subject to condicao_mag_tesao_chave_1 {(i,j) in och}:
  Vqdr[i] - Vqdr[j] <= (Vmax^2-Vmin^2)*(1 - w[i,j]);

subject to condicao_mag_tesao_chave_2 {(i,j) in och}:
  - (Vmax^2-Vmin^2)*(1 - w[i,j]) <= Vqdr[i] - Vqdr[j];
  
subject to limite_fluxo_ativo_chave_1 {(i,j) in och}:
  Pch[i,j] <= (Vmax*Imaxch[i,j])*w[i,j];
  
subject to limite_fluxo_ativo_chave_2 {(i,j) in och}:
  - (Vmax*Imaxch[i,j])*w[i,j] <= Pch[i,j];

subject to limite_fluxo_reativo_chave_1 {(i,j) in och}:
  Qch[i,j] <= (Vmax*Imaxch[i,j])*w[i,j];
  
subject to limite_fluxo_reativo_chave_2 {(i,j) in och}:
  - (Vmax*Imaxch[i,j])*w[i,j] <= Qch[i,j];

subject to condicao_2_radialidade:
  card(ol) + sum{(i,j) in och}(w[i,j]) = card(ob) - 1;
  
subject to limite_magnitude_tensão {i in ob}:
  Vmin^2 <= Vqdr[i] <= Vmax^2;
	
subject to limite_magnitude_corrente {(i,j) in ol}:
  0 <= Iqdr[i,j] <= Imax[i,j]^2;

  
