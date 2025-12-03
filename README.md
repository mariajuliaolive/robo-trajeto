Claro! Aqui está um **README.md completo, organizado, bonito e profissional**, já pensado para colocar no GitHub do seu projeto **Robô Móvel Diferencial com Evasão de Obstáculos + Node-RED + MQTT + PyBullet**.

Se quiser, eu também posso gerar **versão em inglês**, **com imagens**, **com badges do GitHub**, ou **com GIF da simulação**.

---

# 📌 **README.md — Robô Móvel com Evasão de Obstáculos (PyBullet + Node-RED + MQTT)**

## 🤖 **Descrição do Projeto**

Este projeto implementa a simulação completa de um **robô móvel diferencial** com dois motores, sensores ultrassônicos e controle reativo baseado em PID para **desvio de obstáculos em tempo real**.

A simulação é feita no **PyBullet**, o controle e as métricas são enviadas via **MQTT**, e a interface gráfica é construída no **Node-RED**, exibindo telemetria, gráficos e eventos do robô.

O objetivo principal é demonstrar:

* **Controle reativo**
* **Feedback sensorial direto**
* **Evasão de obstáculos em ambiente dinâmico**
* **Monitoramento em tempo real via Node-RED**
* **Coleta e envio de métricas de desempenho**

---

## 🚀 **Funcionalidades Implementadas**

### 🧭 **Robô móvel diferencial**

* Dois motores com controle independente
* Cinemática diferencial
* Controle por velocidade (PWM simulado)

### 👁️ **Sensores simulados**

* Sensores ultrassônicos (frontal e laterais)
* Ruído e latência simulados
* Leitura em tempo real para o controle PID

### 🧱 **Ambiente com obstáculos**

* Obstáculos extensos (10–30% da área)
* Colisões físicas reais


### 🧮 **Controle**

* PID de desvio baseado no **erro lateral**
* Ajuste diferencial de velocidade dos motores
* Comportamento emergente sem planejamento global

---

## 📊 **Métricas monitoradas e enviadas ao Node-RED**

O código publica via MQTT:

| Métrica                               | Descrição                                       |
| ------------------------------------- | ----------------------------------------------- |
| **colisoes**                          | Número total de colisões                        |
| **dist_sem_impacto**                  | Distância percorrida sem impactos               |
| **tempo_reacao_s**                    | Tempo de reação após detectar obstáculo         |
| **erro_lat_medio**                    | Erro médio lateral                              |
| **erro_pos_medio**                    | Erro médio até o objetivo                       |
| **tempo_estabilizacao_s**             | Tempo até estabilizar no alvo                   |
| **energia_total**                     | Energia total consumida (≈ torque × velocidade) |
| **overshoot_angular_max**             | Maior overshoot angular                         |
| **patos_coletados / cubos_coletados** | Quantidade de objetos coletados                 |
| **pos**                               | Posição x,y                                     |
| **motor**                             | Velocidade dos motores                          |
| **sensores**                          | Leitura dos sensores                            |
| **evento_colisao**                    | Flag de colisão                                 |

---

## 🛠️ **Tecnologias utilizadas**

* **Python 3**
* **PyBullet** (simulação física)
* **Node-RED** (dashboard em tempo real)
* **MQTT / Mosquitto**
* **JSON**
* **Controle PID**

---

## 🗂️ **Arquitetura do projeto**

```
/projeto
│
├── main.py              # Simulação PyBullet + controle + métricas
├── dashboard_fluxo.json            # Dashboard do Node-RED
├── README.md             # Este arquivo

```

---

## ⚙️ **Como executar**

### 1️⃣ Instale as dependências

```bash
pip install pybullet paho-mqtt
```

### 2️⃣ Inicie o broker MQTT (Mosquitto)

```bash
mosquitto
```

### 3️⃣ Rode a simulação

```bash
python robot.py
```

### 4️⃣ Importe o fluxo no Node-RED

* Abra **[http://localhost:1880](http://localhost:1880)**
* Menu » Import
* Cole o conteúdo do arquivo **flows.json**

O dashboard ficará disponível em:

```
http://localhost:1880/ui
```

---

## 📺 **Dashboard Node-RED**

O painel exibe:

* Sensores ultrassônicos
* Velocidade dos motores
* Posição do robô
* Contador de colisões
* Contador de objetos coletados
* Gráficos das métricas
* Logs de eventos (colisão, estabilização, coleta, etc.)

---

## 🧪 **Lógica de Evasão**

Controle baseado em:

```
erro = (sensor_direita - sensor_esquerda)
controle = Kp * erro
vel_esquerda  = base - controle
vel_direita   = base + controle
```

Inclui:

✔ Ruído
✔ Atraso
✔ PID (ou proporcional simples)
✔ Overshoot natural
✔ Retorno à trajetória

---

## 📈 **Resultados Observados**

* Comportamento emergente coerente
* Desvio eficiente dos obstáculos
* Trajetória suavizada pelo controle diferencial
* Métricas registradas para análise
* Bom desempenho mesmo com ruído sensorial



---

## 👤 **Equipe**

**Maria Júlia**
**Rafael Diniz**
**Labelle Candido**



