# Importando as bibliotecas
import serial
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Declarando a porta serial
SERIAL_PORT = 'COM5'

# Declarando a velocidade de leitura, tem que ser a mesma do Arduino
SERIAL_RATE = 115200

# Criando um dicionário para armazenar os dados
dataList = {"Tempo [s]"    :[], 
            "Setpoint"    :[], 
            "Valor do Erro"        :[],
            "Valor Atual" :[],
            "Controle PWM"   :[]}
# Guardando as chaves do dicionário em um vetor
Keys = list(dataList.keys())

# Criando função para animar os gráficos
def animate(i, dataList, ser):
    reading = ser.readline().decode('utf-8')  # decodificando a porta serial, importante ressaltar que neste
                                              # caso foi utilizado a utf-8, mas no tutorial foi utilizado ASCII
    
    try:
        reading = [float(value) for value in reading.strip().split(',')]  # Repassa dos dados para float
        if len(reading) == len(Keys):  # Verifica se o vetor recebido possui a mesma quantidade de chaves
            for i in range(len(reading)):
                dataList[Keys[i]].append(reading[i])  # Guarda os valores no dicionário

    except ValueError:  # Ignora erros para não travar o programa
        pass

    # Com o intuito de não utilizar muita memória do computador, limitei o armazenamento para 50 dados
    for key in Keys:
        dataList[key] = dataList[key][-50:]
    
    ax.clear()  # Limpa o plot

    # Informações do gráfico
    ax.plot(dataList["Tempo [s]"], dataList["Setpoint"], label=f"Setpoint = {dataList['Setpoint'][-1]}", color='c', linestyle='--', linewidth=5)
    ax.plot(dataList["Tempo [s]"], dataList["Valor Atual"], label=f"Valor Atual = {dataList['Valor Atual'][-1]}", color='m')
    ax.plot(dataList["Tempo [s]"], dataList["Valor do Erro"], label=f"Valor do Erro = {dataList['Valor do Erro'][-1]}", color='y')
  

    # Formatando o gráfico
    ax.set_xlabel("Tempo [s]")
    ax.set_ylabel("Valores")
    ax.set_title("Saída do Arduino")
    ax.grid(axis="y")
    ax.legend(loc="best")

# Definindo o tamanho da fig
fig, ax = plt.subplots(figsize=(10, 6))

# IComeça a comunicação serial
ser = serial.Serial(SERIAL_PORT, SERIAL_RATE, timeout=0.1)
time.sleep(2)

# Cria a animação
ani = animation.FuncAnimation(fig, animate, fargs=(dataList, ser), interval=0.1, frames=144)

plt.tight_layout()
plt.show()

ser.close()