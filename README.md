# Inversor Trifásico com Controlo Digital ⚡

![Status](https://img.shields.io/badge/Status-Validação_de_Controlo-brightgreen)
![Platform](https://img.shields.io/badge/Plataforma-STM32-blue)
![Hardware](https://img.shields.io/badge/Hardware-KiCad-orange)

## 📌 Sobre o Projeto
Este repositório contém o firmware, o desenho de hardware e as simulações referentes ao desenvolvimento de um **Inversor Trifásico com Controlo Digital**, concebido no âmbito da disciplina de Projeto Integrador. 

O objetivo principal deste sistema é ultrapassar as limitações dos sistemas de controlo analógico (como a suscetibilidade ao ruído e restrições na precisão), transferindo a lógica de comutação e modulação para o domínio digital através de um microcontrolador **STM32 NUCLEO-H755ZI-Q**.

## ✨ Características Principais
* **Retificador Ativo (AFE):** Regulação de tensão num barramento de 400V DC com fator de potência unitário.
* **Inversor Trifásico:** Modulação por Largura de Pulso Sinusoidal (SPWM) operando a uma frequência de 25 kHz com injeção de *dead-time* por hardware.
* **Controlo de Motor:** Estratégia de controlo escalar (V/f constante) para regulação estável da velocidade angular.
* **Arranque Suave (Soft-Start):** Circuito de mitigação de correntes de *inrush* na pré-carga do barramento DC.
* **Telemetria:** Monitorização remota de dados (tensões, correntes e temperatura) via UART em formato JSON, integrada numa *dashboard* Node-RED.

## 📂 Estrutura do Repositório

| Diretório | Descrição |
| :--- | :--- |
| `Firmware/` | Código-fonte em C/C++ desenvolvido para o microcontrolador STM32H755ZI-Q (projeto STM32CubeIDE). |
| `Hardware/` | Ficheiros de desenho KiCad (esquemáticos, layout da PCB e ficheiros Gerber) das placas de controlo e potência. |
| `Simulacoes/` | Esquemas e testes dinâmicos criados no software PSIM para validação teórica dos blocos de potência. |
| `Telemetria/` | Ficheiros de configuração JSON para a interface gráfica no Node-RED. |
| `Documentacao/` | Relatório técnico completo, *datasheets* de componentes e o cartaz académico do projeto. |

## 🛠️ Tecnologias e Ferramentas Utilizadas
* **Hardware Digital:** Microcontrolador STMicroelectronics STM32 NUCLEO-H755ZI-Q.
* **Sensores e Isolamento:** Transdutores de corrente LEM LA100-P e *gate drivers* optoacoplados HCPL-3120.
* **Software/Ambientes:** STM32CubeIDE, KiCad EDA, PSIM, Node-RED.

## ⚠️ Aviso de Segurança
**PERIGO - ALTA TENSÃO:** Este projeto lida com tensões perigosas (entrada da rede elétrica e barramento DC de 400V). A replicação física e os testes na fase de potência requerem equipamento de proteção adequado e a supervisão de profissionais qualificados. Certifique-se de que os condensadores do barramento estão totalmente descarregados antes de manusear as placas.

## 👥 Equipa e Agradecimentos
**Desenvolvedores:** Afonso Carvalho, Pedro Dias, Luís Gonçalves e Nuno Costa.
**Orientação:** Prof. Luís Barros

Projeto desenvolvido com o apoio do **GEPE** (Grupo de Eletrónica de Potência e Energia) e do **NEEEICUM** (Núcleo de Estudantes de Engenharia Eletrónica Industrial e Computadores) da Universidade do Minho, Guimarães.
