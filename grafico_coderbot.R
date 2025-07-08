library("ggplot2")


#-----Input-----
#copio i dati delle traiettorie così come sono
args <- commandArgs(trailingOnly = TRUE)

testo_t_prevista <- args[1]
testo_t_seguita <- args[2]

# Pulizia degli spazi invisibili (non-breaking space)
testo_t_prevista <- gsub("\u00A0", " ", testo_t_prevista, fixed = TRUE)
testo_t_seguita  <- gsub("\u00A0", " ", testo_t_seguita,  fixed = TRUE)


#-----Funzioni Utili------

# Funzione generale per parsare una stringa di coordinate (x, y)
parsa_coordinate <- function(testo) {
  # Trova tutte le sottostringhe tipo "(x, y)"
  punti <- unlist(regmatches(testo, gregexpr("\\([^\\)]+\\)", testo)))

  # Converte ogni coppia in vettore numerico
  coordinate <- lapply(punti, function(p) {
    valori <- strsplit(gsub("[()]", "", p), ",")[[1]]
    as.numeric(trimws(valori))
  })

  return(coordinate)  # Ritorna lista di c(x, y)
}

# Funzione per estrarre due vettori separati (x, y) da una lista di coordinate
estrai_xy <- function(coppie) {
  x <- sapply(coppie, function(p) p[1])  # Estrae il primo elemento di ogni coppia
  y <- sapply(coppie, function(p) p[2])  # Estrae il secondo elemento
  return(list(x = x, y = y))  # Ritorna come lista con x e y
}


# Funzione per creare un grafico con due traiettorie
genera_grafico_traiettorie <- function(traiettoria_prevista, traiettoria_seguita, label1 = "Stimata", label2 = "Reale") {

  # Estrae x e y separati per entrambe le traiettorie
  xy1 <- estrai_xy(traiettoria_prevista)
  xy2 <- estrai_xy(traiettoria_seguita)

  # Calcola la lunghezza massima tra le due traiettorie
  n1 <- length(xy1$x)
  n2 <- length(xy2$x)
  n_max <- max(n1, n2)

  # Riempi i vettori più corti con NA per pareggiare le lunghezze
  xy1$x <- c(xy1$x, rep(NA, n_max - n1))
  xy1$y <- c(xy1$y, rep(NA, n_max - n1))
  xy2$x <- c(xy2$x, rep(NA, n_max - n2))
  xy2$y <- c(xy2$y, rep(NA, n_max - n2))

  # Crea un unico data.frame per il grafico
  dati <- data.frame(
    x = c(xy1$x, xy2$x),
    y = c(xy1$y, xy2$y),
    tipo = rep(c(label1, label2), each = n_max)
  )

  # Ultimi punti per segnare la fine delle traiettorie
  fine <- data.frame(
    x = c(tail(na.omit(xy1$x), 1), tail(na.omit(xy2$x), 1)),
    y = c(tail(na.omit(xy1$y), 1), tail(na.omit(xy2$y), 1)),
    tipo = c(label1, label2)
  )

  # Crea il grafico ggplot
  ggplot(dati, aes(x = x, y = y, color = tipo)) +
    geom_point(shape = 1, size = 2) +          # Punti
    geom_line(na.rm = TRUE, size = 0.5) +                  # Linee, ignora i NA
    geom_point(data = fine, aes(x = x, y = y),            # Punto finale
               shape = 8, size = 3, stroke = 2) +         # Stella
    geom_hline(yintercept = 0, color = "black", linewidth = 1) +  # 👈 Asse X nero
    geom_vline(xintercept = 0, color = "black", linewidth = 1) +  # 👈 Asse Y nero
    scale_color_manual(values = setNames(c("green", "orange"), c(label1, label2))) +
    labs(title = "Confronto traiettorie",
         x = "X", y = "Y", color = "Tipo") +
    scale_x_continuous(breaks = seq(-900, 900, by = 200)) +  # 👈 Asse X ogni 100
  scale_y_continuous(breaks = seq(-900, 900, by = 200)) +  # 👈 Asse Y ogni 100
  coord_cartesian(xlim = c(-900, 900), ylim = c(-900, 900)) +  # 👈 Mantiene il quadrato
    theme_minimal()
}



#-----Inserimento Dati-----
# Converte i testi in coordinate (liste di c(x, y))
coordinate_previste <- parsa_coordinate(testo_t_prevista)
coordinate_seguite <- parsa_coordinate(testo_t_seguita)


#-----Visualizzazione Grafico-----
# Crea e mostra il grafico
genera_grafico_traiettorie(coordinate_previste, coordinate_seguite)
