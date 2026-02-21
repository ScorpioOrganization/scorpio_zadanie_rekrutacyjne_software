# Projekt Scorpio - zadanie rekrutacyjne do działu Software

> **Uwaga!** Przed przystąpieniem do realizacji zadania przeczytaj **całe** README i pamiętaj, żeby wypełnić [formularz rekrutacyjny]([TODO: uzupełnić]).

## Spis treści
- [Informacje ogólne](#informacje-ogólne)
- [Zadania do wykonania](#zadania-do-wykonania)
- [Wskazówki i przydatne linki](#wskazówki-i-przydatne-linki)

## Informacje ogólne
Zadanie wymaga instalacji ROS2 Humble i silnie zalecamy korzystania w tym celu z Ubuntu 22.04. Jeżeli nie chcesz instalować na swoim komputerze nowego systemu, to można zrealizować zadanie np. używając [devcontainera](https://code.visualstudio.com/docs/devcontainers/containers).

Całość zadania należy wykonać w języku C++, zgodnie ze standardem C++17.

Rozwiązane zadanie należy umieścić w **publicznym** repozytorium (np. GitHub) i przesłać linka do tego repozytorium na mail projekt@scorpio.pwr.edu.pl. Ewentualne pytania lub wątpliwości co do treści zadania można kierować na tego samego maila. Zadania przyjmujemy do 29.03.2026 do końca dnia.

## Zadania do wykonania 
W tej części znajdziesz treści poszczególnych zadań.

Pamiętaj, że zadanie służy sprawdzeniu wielu umiejętności - nie tylko programowania i znajomości algorytmów -  więc nawet w przypadku zrealizowania tylko części z poniższych punktów, zachęcamy do przesłania rozwiązania. Postępy w zadaniu powinny być udokumentowane w repozytorium na GitHubie (po każdym etapie zadania powinien zostać stworzony nowy commit).

1. **Konfiguracja projektu i zbudowanie przykładu**
  - Należy zainstalować ROS2 Humble zgodnie z [oficjalną instrukcją](https://docs.ros.org/en/humble/Installation.html) na odpowiedni system.
  - Należy [utworzyć workspace ROS-owy](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html), i sklonować do folderu `src` to repozytorium.
  - Należy zbudować workspace, zsourceować go, i uruchomić przykładowego node'a za pomocą `ros2 run scorpio_zadanie_rekrutacyjne_software cube_detector` (więcej informacji o każdym z tych kroków znajdziecie w [tym tutorialu](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)) 
  - Node udostępnia dwa topici:
    - `/cube_detector/input` - topic typu `std_msgs/msg/String`, na który można wysyłać wiadomości tekstowe, które będą logowane, oraz wysyłane na `/cube_detector/output`.
    - `/cube_detector/output` - topic typu `std_msgs/msg/String`, na który node wysyła wiadomości tekstowe, które otrzymał na poprzednim topicu.
  - Należy przetestować działanie node'a komendami `ros2 topic pub` oraz `ros2 topic echo`.

  > **Wskazówka!** Możesz użyć tego przykładowego node'a jako bazy do realizacji kolejnych punktów zadania, ale nie jest to konieczne. Jeżeli wolisz, możesz stworzyć własny node lub node'y od zera, które będą realizować poniższe funkcjonalności.

## Wskazówki i przydatne linki

- Zadanie rekrutacyjne można oddać niepełne.
- Rozwiązane zadanie należy umieścić w **publicznym** repozytorium (np. GitHub) i przesłać linka do tego repozytorium na mail projekt@scorpio.pwr.edu.pl. Ewentualne pytania lub wątpliwości co do treści zadania można kierować na tego samego maila. Zadania przyjmujemy do 29.03.2026 do końca dnia.

**Jeżeli będziesz miał jakiekolwiek wątpliwości i problemy z zadaniem śmiało skontaktuj się z nami na maila projekt@scorpio.pwr.edu.pl! Powodzenia :)**
