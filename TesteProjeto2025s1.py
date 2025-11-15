from controller import Supervisor
import math
import random

def pega_tempo_passos(robo):
    return int(robo.getBasicTimeStep()) or 32

class epuck_busca_caixa(Supervisor):
    def __init__(self):
        super().__init__()

        self.tempo_passos = pega_tempo_passos(self)

        self.motor_esq = self.getDevice('left wheel motor')
        self.motor_dir = self.getDevice('right wheel motor')
        self.motor_esq.setPosition(float('inf'))
        self.motor_dir.setPosition(float('inf'))
        self.motor_esq.setVelocity(0.0)
        self.motor_dir.setVelocity(0.0)

        self.sens_ir = [self.getDevice(f'ps{i}') for i in range(8)]
        for sen in self.sens_ir:
            sen.enable(self.tempo_passos)

        self.node_robo = self.getSelf()

        self.dados_caixas = []
        self.lim_massa_especial = 0.06
        self.tol_massa = 1e-4

        self.tempo_lim_preso = 4.0 # seg
        self.lim_pos = 0.01 # metros
        self.dur_recuo = 0.5
        self.dur_giro_desviar = 1.0
        self.vel_recuo = -2.0 
        self.vel_giro_esq = 2.0
        self.vel_giro_dir = -2.0 

        print("Lendo massas das caixas...")
        for i in range(1, 21):
            nome_caixa = f"CAIXA{i:02d}"
            node_caixa = self.getFromDef(nome_caixa)
            if node_caixa is None:
                continue

            campo_massa = node_caixa.getField("mass")
            massa = campo_massa.getSFFloat() if campo_massa else -1.0

            caixa_especial = abs(massa - self.lim_massa_especial) < self.tol_massa
            self.dados_caixas.append((node_caixa, massa, caixa_especial))

            info = f"DEF={nome_caixa}, massa={massa}"
            if caixa_especial:
                info += " <--- ESPECIAL (por massa)"
            print(" ->", info)

        self.estado_atual = "BUSCANDO"

        self.vel_maxima = 6.28
        self.vel_frente = 0.5 * self.vel_maxima
        self.vel_giro = 0.4 * self.vel_maxima
        self.vel_rodar = 0.4 * self.vel_maxima

        self.lim_colisao = 0.115  # 11.5 cm
        self.tempo_cooldown_colisao = 0.3
        self.ultimo_tempo_colisao = -999.0

        self.estado_mov = "FRONTAL"
        self.tempo_fim_mov = 0.0
        self.dir_giro = 0  # -1 esq, +1 dir

        self.ultima_pos_conhecida = None
        self.tempo_inicio_preso = 0.0
        self.desviando = False
        self.tempo_fim_desvio = 0.0
        self.fase_desvio_preso = "NENHUMA"

    def pega_tempo_atual(self):
        return self.getTime()

    def pega_coord_robo(self):
        return self.node_robo.getField('translation').getSFVec3f()

    def calcular_dist(self, ponto1, ponto2):
        return math.sqrt((ponto1[0] - ponto2[0]) ** 2 + (ponto1[1] - ponto2[1]) ** 2 + (ponto1[2] - ponto2[2]) ** 2)

    def ajustar_vels_rodas(self, vel_esq, vel_dir):
        self.motor_esq.setVelocity(vel_esq)
        self.motor_dir.setVelocity(vel_dir)

    def att_estado_mov(self):
        tempo_atual = self.pega_tempo_atual()
        if tempo_atual < self.tempo_fim_mov:
            return

        if random.random() < 0.7:
            self.estado_mov = "FRONTAL"
            duracao = random.uniform(1.0, 3.0)
            self.dir_giro = 0
        else:
            self.estado_mov = "GIRANDO"
            duracao = random.uniform(0.5, 1.0)
            self.dir_giro = random.choice([-1, 1])

        self.tempo_fim_mov = tempo_atual + duracao

    def exec_mov_com_evasao_fraca(self):
        self.att_estado_mov()

        if self.estado_mov == "FRONTAL":
            vel_esq = self.vel_frente
            vel_dir = self.vel_frente
        else:
            dir = self.dir_giro
            vel_esq = dir * self.vel_giro
            vel_dir = -dir * self.vel_giro

        val_sens = [sen.getValue() for sen in self.sens_ir]
        lim_emer = 350.0  # se for muito perto

        frente_dir = val_sens[0]
        dir = val_sens[1]
        tras_dir = val_sens[2]
        tras_esq = val_sens[5]
        esq = val_sens[6]
        frente_esq = val_sens[7]

        frente_prox = frente_dir > lim_emer or frente_esq > lim_emer
        esq_prox = esq > lim_emer or tras_esq > lim_emer
        dir_prox = dir > lim_emer or tras_dir > lim_emer

        if frente_prox or esq_prox or dir_prox:
            if frente_prox:
                if frente_dir > frente_esq:
                    vel_esq = -self.vel_giro
                    vel_dir = self.vel_giro
                else:
                    vel_esq = self.vel_giro
                    vel_dir = -self.vel_giro
            elif esq_prox:
                vel_esq = self.vel_giro
                vel_dir = -self.vel_giro * 0.5
            elif dir_prox:
                vel_esq = -self.vel_giro * 0.5
                vel_dir = self.vel_giro

        self.ajustar_vels_rodas(vel_esq, vel_dir)

    def loc_caixa_mais_prox(self):
        pos_robo = self.pega_coord_robo()
        dist_min = float('inf')
        caixa_mais_prox = None
        massa_mais_prox = None
        especial_mais_prox = False
        nome_def_mais_prox = None

        for node_caixa, massa, caixa_especial in self.dados_caixas:
            pos_caixa = node_caixa.getField('translation').getSFVec3f()
            dist = self.calcular_dist(pos_robo, pos_caixa)
            if dist < dist_min:
                dist_min = dist
                caixa_mais_prox = node_caixa
                massa_mais_prox = massa
                especial_mais_prox = caixa_especial
                nome_def_mais_prox = node_caixa.getDef()

        return caixa_mais_prox, massa_mais_prox, especial_mais_prox, dist_min, nome_def_mais_prox

    def avaliar_colisao(self):
        tempo_atual = self.pega_tempo_atual()
        if tempo_atual - self.ultimo_tempo_colisao < self.tempo_cooldown_colisao:
            return

        node_caixa, massa, caixa_especial, dist, nome_def = self.loc_caixa_mais_prox()
        if node_caixa is None:
            return

        if dist < 0.20:
            # print(f"Perto de {nome_def}: dist={dist:.3f} m, massa={massa}, especial={caixa_especial}")

            if dist < self.lim_colisao:
                self.ultimo_tempo_colisao = tempo_atual
                if caixa_especial:
                    print(f">>> COLISÃO ESPECIAL em {nome_def}! dist={dist:.3f}, massa={massa}")
                    self.estado_atual = "GIRANDO" # Mudar para o estado de giro intencional
                else:
                    print(f"Colisão não especial em {nome_def}: dist={dist:.3f}, massa={massa}")


    def girar_para_sempre(self):
        self.ajustar_vels_rodas(self.vel_rodar, -self.vel_rodar)

    def exec(self):
        print("Controlador epuck_busca_caixa iniciado.")
        while self.step(self.tempo_passos) != -1:
            tempo_atual = self.pega_tempo_atual()

            # ficando preso
            if self.desviando:
                if tempo_atual < self.tempo_fim_desvio:
                    pass
                else:
                    if self.fase_desvio_preso == "RECUANDO":
                        print(f"Virando em {tempo_atual:.2f}s.")
                        self.ajustar_vels_rodas(self.vel_giro_esq, self.vel_giro_dir)
                        self.fase_desvio_preso = "VIRANDO"
                        self.tempo_fim_desvio = tempo_atual + self.dur_giro_desviar
                    elif self.fase_desvio_preso == "VIRANDO":
                        print(f"desvio de desobstrução concluída em {tempo_atual:.2f}s. Retomando movimento normal.")
                        self.desviando = False
                        self.fase_desvio_preso = "NENHUMA"
                        self.ultima_pos_conhecida = None # Reset para pegar uma nova posição inicial
                        self.tempo_inicio_preso = 0.0
                        self.ajustar_vels_rodas(0.0, 0.0)
            else:
                # não esta desviando
                if self.estado_atual == "BUSCANDO":
                    pos_atual = self.pega_coord_robo()
                    if self.ultima_pos_conhecida is None:
                        self.ultima_pos_conhecida = pos_atual
                        self.tempo_inicio_preso = tempo_atual
                    distancia = self.calcular_dist(pos_atual, self.ultima_pos_conhecida)
                    if distancia < self.lim_pos:
                        # nao moveu muito
                        if tempo_atual - self.tempo_inicio_preso >= self.tempo_lim_preso:
                            # preso
                            print(f"Robô preso! Iniciando desvio de desobstrução em {tempo_atual:.2f}s.")
                            self.desviando = True
                            self.fase_desvio_preso = "RECUANDO"
                            self.tempo_fim_desvio = tempo_atual + self.dur_recuo
                            # recua
                            self.ajustar_vels_rodas(self.vel_recuo, self.vel_recuo)
                        else:
                            # preso por pouco tempo, continua tentando
                            self.exec_mov_com_evasao_fraca()
                            self.avaliar_colisao()
                    else:
                        # movendo normal
                        self.ultima_pos_conhecida = pos_atual
                        self.tempo_inicio_preso = tempo_atual
                        self.exec_mov_com_evasao_fraca()
                        self.avaliar_colisao()
                elif self.estado_atual == "GIRANDO":
                    self.girar_para_sempre()

if __name__ == "__main__":
    epuck_busca_caixa().exec()
