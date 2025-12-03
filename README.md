
🤖 Robô Móvel com Evasão de Obstáculos – PyBullet
Este projeto simula um **robô móvel diferencial** capaz de navegar em um ambiente com obstáculos, utilizando sensores ultrassônicos virtuais para detectar colisões e ajustar sua trajetória em tempo real.

O robô foi desenvolvido usando **PyBullet**, com lógica de movimentação suave, velocidade reduzida e curvatura aprimorada para tornar o comportamento mais natural.



 🚀 Funcionalidades

* Robô móvel com dois motores de tração (modelo diferencial).
* Sensores ultrassônicos virtuais frontais e laterais.
* Desvio de obstáculos automático.
* Curvas suaves e movimentação mais lenta e realista.
* Ambiente com obstáculos gerados no PyBullet.
* Controle baseado em análise de distância e ajustes dinâmicos de direção.

---

## 📁 Estrutura do Projeto

```
📦 robo-trajeto
 ┣ 📜 main.py        # Arquivo principal da simulação
 ┣ 📜 robot.py       # Classe do robô e sensores
 ┣ 📜 controller.py  # Lógica de movimentação e evasão
 ┣ 📜 environment.py # Ambiente, obstáculos, chão
 ┗ 📜 README.md
```

---

## ▶️ Como Executar

### 1. Instale as dependências:

```bash
pip install pybullet numpy
```

### 2. Execute o simulador:

```bash
python main.py
```

A janela do PyBullet abrirá automaticamente mostrando o robô navegando.




