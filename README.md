# Controle de Temperatura para Sensor LWIR — CTEx

Sistema de compensação térmica para sensor infravermelho de onda longa (LWIR), desenvolvido durante estágio no **Laboratório de Optrônica e Sensores (LOS)** do Centro Tecnológico do Exército (CTEx).

---

## Problema

Sensores LWIR aquecem durante as medições a partir de certa temperatura, o que compromete a precisão das leituras. Este projeto resolve esse problema mantendo o sensor em uma faixa de temperatura estável por meio de controle ativo com célula Peltier.

---

## Arquitetura do Sistema

```
Sensor de temperatura → Arduino R4 Minima → TPS54200 → Célula Peltier → Sensor LWIR
```

A solução evoluiu em duas etapas:

**Etapa 1 — PWM com ponte H**
Abordagem inicial usando modulação por largura de pulso (PWM) para acionar a célula Peltier via ponte H. Funcionou parcialmente, mas o switching do PWM introduzia ruído elétrico que interferia nas leituras do sensor.

**Etapa 2 — Controle de corrente constante (solução final)**
Migração para o conversor TPS54200, que fornece corrente constante à célula Peltier, eliminando as variações abruptas do PWM. O resultado foi redução significativa de ruído e maior estabilidade térmica do sistema.

---

## Hardware

### Componentes principais

| Componente | Função |
|---|---|
| Arduino R4 Minima | Controle e lógica do sistema |
| TPS54200 | Conversor buck — fornece corrente constante |
| Célula Peltier | Elemento de resfriamento termoelétrico |
| Sensor de temperatura | Monitoramento da temperatura do sensor LWIR |

### PCB

A placa foi projetada no **KiCad** e inclui:

- Circuito de controle baseado no TPS54200
- Shield para integração direta com o Arduino R4 Minima
- Layout otimizado para redução de interferência eletromagnética

---

## Firmware

O código foi desenvolvido em C++ para Arduino R4 Minima e implementa:

- Leitura contínua do sensor de temperatura
- Algoritmo de controle para acionamento do TPS54200
- Monitoramento serial para debug e validação

---

## Estrutura do Repositório

```
├── 10khzTPS54200.ino          # Firmware Arduino — controle de corrente constante
├── AprendendoKiCad.kicad_pcb  # Layout da PCB
├── AprendendoKiCad.kicad_sch  # Esquemático
├── AprendendoKiCad.kicad_pro  # Arquivo de projeto KiCad
├── New_Library.kicad_sym      # Biblioteca de símbolos customizada
├── conectorarduino1.kicad_sym # Símbolo do conector Arduino
└── README.md
```

---

## Contexto

Projeto desenvolvido durante estágio no **CTEx — Centro Tecnológico do Exército**, Laboratório de Optrônica e Sensores (LOS), de agosto a novembro de 2025.

Resultados do projeto contribuíram para a redação de um artigo científico do laboratório.

---

## Tecnologias

![KiCad](https://img.shields.io/badge/KiCad-314CB0?style=flat&logo=kicad&logoColor=white)
![Arduino](https://img.shields.io/badge/Arduino-00979D?style=flat&logo=arduino&logoColor=white)
![C++](https://img.shields.io/badge/C++-00599C?style=flat&logo=cplusplus&logoColor=white)

---

## Autora

**Raquel Belchior**

Estudante de Engenharia Eletrônica — Instituto Militar de Engenharia (IME)
[LinkedIn](https://linkedin.com/in/raquel-belchior-b03907226) · [GitHub](https://github.com/raquelbelchior1)
