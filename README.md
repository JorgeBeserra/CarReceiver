
CarReceiver - README

Objetivo
- CarReceiver é um conjunto de scripts e infraestrutura para gerenciar recebimento e automação de dados/controle veicular (Kodi-Car, sensores, etc.).
- Foca em atualização segura, versionamento e implantação simples.

Arquitetura (alto nível)
- src/: código principal
- docs/: documentação
- tests/: testes
- configs/: configurações de ambiente/CI
- scripts/: utilitários (update/rollback/backup)
- hardware/kicad/: arquivos de design da PCB/esquemático (KiCad)

Hardware Setup
- Microcontrolador: ESP32
- Transistores de controle: 2x 2N2222 SMDs
- Função: controle de ponte H para motores e LEDs.
- Design de PCB: Utiliza KiCad para esquemático (.kicad_sch) e layout de PCB (.kicad_pcb).

Firmware
- Desenvolvido em C/C++ usando PlatformIO e o framework Arduino.

Como usar (visão rápida)
- Clone o repositório
- Instale dependências (Python 3.11+ para scripts de automação e CLI)
- Configure o ambiente PlatformIO com `platformio.ini`
- Compile e faça upload do firmware para o ESP32
- Rode scripts de atualização conforme necessário

Contribuição
- Use branching adequado (feat/..., fix/...).
- Siga Conventional Commits para mensagens (tipo: feat:, fix:, chore:).

Roadmap Inicial
- MVP de atualização firmware com rollback
- CI/CD com GitHub Actions para build e release do firmware
- Processo de release SemVer

Licença
- MIT licence
