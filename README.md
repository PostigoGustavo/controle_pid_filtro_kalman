# controle_pid_filtro_kalman
Projeto motor DC com filtro de Kalman


![image](https://github.com/user-attachments/assets/e09b50f6-d80d-447b-aa24-615cb38af8d3)

LEGENDA DOS GRÁFICOS

1. Measured vs. Filtered RPM (Primeiro Gráfico)
Curva vermelha (Measured RPM): Representa a medição do RPM com ruído. Percebe-se que no início há uma diferença visível entre o valor medido e o setpoint (valor desejado), mas ela se ajusta rapidamente.
Curva azul (Filtered RPM - Kalman): O filtro de Kalman suavizou os dados medidos, convergindo rapidamente para o valor desejado. A diferença inicial entre a linha vermelha (medida) e azul (filtrada) é visível, mostrando a capacidade do filtro de reduzir o ruído.
Curva verde tracejada (Setpoint): A linha representa o valor de RPM desejado, que é 100 RPM. O filtro de Kalman conseguiu aproximar os valores rapidamente ao setpoint, melhorando a precisão da estimativa em relação à medida.
Conclusão: O filtro de Kalman está funcionando bem para remover o ruído das medições e fornecer uma estimativa precisa da velocidade do motor.

2. Control Signal (PID Output) (Segundo Gráfico)
O gráfico mostra que o sinal de controle (saída do PID) começa alto e decai rapidamente. Isso é esperado em um sistema de controle, onde uma ação inicial forte ajusta o erro e, conforme o erro diminui, o controle se estabiliza em valores baixos.
Após o ajuste inicial, o controle permanece próximo de zero, indicando que o motor já atingiu o valor desejado de RPM.
Conclusão: O controlador PID reagiu adequadamente, com uma ação rápida no início para corrigir o erro e depois estabilizando-se conforme o motor atinge o setpoint.

3. Actual Motor RPM (Terceiro Gráfico)
O RPM do motor começa de 0 e sobe rapidamente até se estabilizar em torno de 100 RPM, conforme esperado.
A resposta é suave, sem oscilações significativas, o que sugere que o sistema está bem sintonizado.
Conclusão: O motor atingiu a velocidade desejada de 100 RPM de forma estável e rápida, sem oscilações indesejadas ou excesso de ruído.
