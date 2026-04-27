# Analisi: Mappa 3D a Risoluzione Variabile (Multi-Resolution Grid)

**Data:** 23 Gennaio 2026  
**Versione:** 1.0  
**Stato:** Analisi Preliminare

---

## 📑 Indice

1. [L'Idea in Breve](#1-lidea-in-breve)
2. [Contesto: Il Tuo Caso d'Uso](#2-contesto-il-tuo-caso-duso)
3. [Approcci Tecnici](#3-approcci-tecnici)
4. [Analisi Pro e Contro](#4-analisi-pro-e-contro)
5. [Impatto sull'Implementazione Attuale](#5-impatto-sullimplementazione-attuale)
6. [Confronto Quantitativo](#6-confronto-quantitativo)
7. [Alternative da Considerare](#7-alternative-da-considerare)
8. [Raccomandazione Finale](#8-raccomandazione-finale)

---

## 1. L'Idea in Breve

L'idea è di creare una mappa 3D con **risoluzione variabile**:
- **Alta risoluzione (voxel piccoli)** vicino al centro/robot
- **Bassa risoluzione (voxel grandi)** verso la periferia

```
┌─────────────────────────────────────────────────────────────────────┐
│                    GRIGLIA A RISOLUZIONE UNIFORME                   │
│    (Implementazione Attuale)                                        │
│                                                                     │
│    ┌──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┬──┐         │
│    ├──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┤         │
│    ├──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┤         │
│    ├──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┤         │
│    ├──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┤         │
│    ├──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┤         │
│    ├──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┤         │
│    ├──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┼──┤         │
│    └──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┴──┘         │
│    Tutti i voxel hanno la stessa dimensione                         │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│                 GRIGLIA A RISOLUZIONE VARIABILE                     │
│    (L'Idea Proposta)                                                │
│                                                                     │
│    ┌──────────────┬───────┬────┬────┬───────┬──────────────┐       │
│    │              │       │    │    │       │              │       │
│    │              │       ├────┼────┤       │              │       │
│    │              ├───────┼────┼────┼───────┤              │       │
│    │              │       │ ⊙ │    │       │              │       │
│    │              ├───────┼────┼────┼───────┤              │       │
│    │              │       │    │    │       │              │       │
│    │              │       ├────┼────┤       │              │       │
│    └──────────────┴───────┴────┴────┴───────┴──────────────┘       │
│    ⊙ = centro (robot). Voxel piccoli vicino, grandi lontano        │
└─────────────────────────────────────────────────────────────────────┘
```

**Motivazione**: Avere una mappa più estesa senza aumentare eccessivamente il numero di voxel e quindi il costo computazionale.

---

## 2. Contesto: Il Tuo Caso d'Uso

### 2.1. Configurazione Attuale

Dalla tua implementazione in `map3d`:

| Parametro | Valore Attuale | Note |
|-----------|----------------|------|
| **Dimensione** | 3×3×2 m | Mappa centrata su `base_link` |
| **Risoluzione** | 0.05 m (5 cm) | Uniforme |
| **Voxel totali** | 60×60×40 = **144,000** | Circa 720 KB per buffer |
| **Update rate** | 10-20 Hz | EDT con algoritmo F&H |
| **Query rate** | 100 Hz | Interpolazione trilineare |
| **Frame** | `base_link` | Mappa solidale al robot |

### 2.2. Perché Vorresti Espandere?

Possibili motivazioni:
1. **Maggiore range di rilevamento** - Vedere ostacoli prima che siano vicini
2. **Pianificazione anticipata** - Decidere traiettorie con più anticipo
3. **Base mobile (futuro)** - Spazio di lavoro più esteso

### 2.3. Vincoli da Rispettare

| Vincolo | Critico | Motivo |
|---------|---------|--------|
| **EDT O(n)** | ✅ Sì | L'algoritmo F&H richiede griglia uniforme |
| **Query O(1)** | ✅ Sì | Interpolazione trilineare a 100 Hz |
| **Double buffer** | ✅ Sì | Lock-free per real-time |
| **Gradiente continuo** | ⚠️ Importante | Per velocità repulsive smooth |

---

## 3. Approcci Tecnici

Esistono diverse tecniche per implementare mappe a risoluzione variabile:

### 3.1. Octree (Struttura Gerarchica)

```
                    ┌───────────────────────┐
                    │    ROOT (Level 0)     │
                    │     1 voxel, N m      │
                    └───────────┬───────────┘
                                │
                    ┌───────────┴───────────┐
                    │    Split in 8 figli   │
                    └───────────┬───────────┘
                    ┌───┬───┬───┼───┬───┬───┐
                    ▼   ▼   ▼   ▼   ▼   ▼   ▼
              ┌───────┐                   ┌───────┐
              │ L1    │       ...         │ L1    │
              │(solo  │                   │ (può  │
              │ se    │                   │  essere│
              │ serve)│                   │  split)│
              └───────┘                   └───────┘
```

**Caratteristiche:**
- Suddivide ricorsivamente solo dove serve
- Memoria adattiva (poca dove la scena è vuota o lontana)
- Famoso esempio: **Octomap** (usato in ROS)

**Problemi per il tuo caso:**
- ❌ L'EDT classico (F&H) **non funziona direttamente** su octree
- ❌ Query più lente: O(log n) invece di O(1)
- ❌ Gradiente può essere discontinuo ai confini tra livelli
- ⚠️ Complessità implementativa significativa

### 3.2. Multi-Level Grid (Cascade)

```
┌─────────────────────────────────────────────────────────────────────┐
│                    LIVELLO 0 (dettaglio alto)                       │
│    Zona: -1.5m a +1.5m     Risoluzione: 0.025m    Voxel: 120³       │
│                                                                     │
│         ┌──────────────────────────────────────────┐               │
│         │ ┌────────────────────────────────────┐   │               │
│         │ │                                    │   │               │
│         │ │              ROBOT                 │   │               │
│         │ │                ⊙                  │   │               │
│         │ │                                    │   │               │
│         │ └────────────────────────────────────┘   │               │
│         └──────────────────────────────────────────┘               │
│                                                                     │
│    LIVELLO 1 (dettaglio basso)                                      │
│    Zona: -3m a +3m (escluso L0)  Risoluzione: 0.1m   Voxel: 60³    │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

**Caratteristiche:**
- Due (o più) griglie uniformi sovrapposte
- Ogni griglia ha il proprio EDT
- Query sceglie il livello in base alla posizione

**Problemi:**
- ⚠️ **Discontinuità ai confini** tra livelli
- ⚠️ Gestione delle transizioni
- ✅ Ogni livello è una griglia uniforme → EDT funziona

### 3.3. Griglia Radiale/Sferica

```
┌─────────────────────────────────────────────────────────────────────┐
│                    GRIGLIA IN COORDINATE SFERICHE                   │
│                                                                     │
│                           ╱       ╲                                │
│                         ╱     ⊙     ╲                              │
│                        │    robot    │                             │
│                         ╲           ╱                              │
│                           ╲       ╱                                │
│                                                                     │
│    Voxel: (r, θ, φ)                                                │
│    - δr piccolo vicino, grande lontano                              │
│    - Risoluzione angolare costante                                  │
└─────────────────────────────────────────────────────────────────────┘
```

**Problemi:**
- ❌ Voxel molto deformati
- ❌ EDT in coordinate sferiche è **complesso**
- ❌ Singolarità ai poli

### 3.4. Log-Polar Grid / Esponenziale

```
┌─────────────────────────────────────────────────────────────────────┐
│                    RISOLUZIONE ESPONENZIALE                         │
│                                                                     │
│    Distanza dal centro → Dimensione voxel                           │
│                                                                     │
│    r = 0.0m → 0.02m (2 cm)                                         │
│    r = 0.5m → 0.04m                                                 │
│    r = 1.0m → 0.08m                                                 │
│    r = 2.0m → 0.16m                                                 │
│    r = 4.0m → 0.32m                                                 │
│                                                                     │
│    Formula: δ(r) = δ₀ × 2^(r/r₀)                                   │
└─────────────────────────────────────────────────────────────────────┘
```

**Problemi:**
- ❌ Griglia non uniforme → EDT non applicabile direttamente
- ❌ Interpolazione complessa

---

## 4. Analisi Pro e Contro

### 4.1. Tabella Comparativa

| Aspetto | Griglia Uniforme (Attuale) | Multi-Resolution |
|---------|---------------------------|------------------|
| **Complessità implementativa** | ✅ Semplice | ❌ Alta |
| **EDT** | ✅ O(n) F&H | ⚠️ Richiede adattamenti |
| **Query** | ✅ O(1) | ⚠️ O(log n) o discontinuità |
| **Memoria per area coperta** | ❌ Scala cubicamente | ✅ Più efficiente |
| **Gradiente continuo** | ✅ Sì (trilinear interp) | ⚠️ Discontinuità possibili |
| **Manutenibilità** | ✅ Codice semplice | ❌ Debug difficile |

### 4.2. Il Problema Fondamentale: EDT

Il cuore del tuo sistema è l'**Euclidean Distance Transform** implementato con l'algoritmo di **Felzenszwalb & Huttenlocher**. Questo algoritmo:

```cpp
// Dal tuo codice: edt_calculator.cpp
// Passage X, Y, Z in sequenza
// RICHIEDE che la griglia sia UNIFORME lungo ogni asse

for (std::size_t z = 0; z < nz; ++z) {
    for (std::size_t y = 0; y < ny; ++y) {
        // EDT 1D lungo X con passo COSTANTE
        edt1d(line_in.data(), static_cast<int>(nx), line_out.data());
    }
}
```

**L'algoritmo F&H non funziona con griglie a risoluzione variabile** perché:
1. L'envelope di parabole assume spaziatura costante
2. La conversione `distanza_voxel → distanza_metri` assume `res` costante
3. La separabilità X→Y→Z assume griglia cartesiana uniforme

### 4.3. Il Problema delle Discontinuità

Con griglie multi-risoluzione, si creano **discontinuità al confine tra zone**:

```
┌───────────────────────────────────────────────────────────────────┐
│            DISCONTINUITÀ AL CONFINE TRA RISOLUZIONI               │
│                                                                   │
│   Distanza                                                        │
│     ↑                  ─────────────────                          │
│     │                 ╱                                           │
│     │    ──────────────           ← Curva ideale (liscia)         │
│     │   ╱                                                         │
│     │──╱                                                          │
│     │ │                                                           │
│     │ ├───────────────────────────────────────────→ Posizione     │
│     │ │                                                           │
│       │← confine hi-res/lo-res                                    │
│                                                                   │
│   Con multi-res:                                                  │
│     ↑           ·                                                 │
│     │          ·─────────────────                                 │
│     │───────────                                                  │
│     │          │                                                  │
│     │          ← SALTO (discontinuità)                            │
└───────────────────────────────────────────────────────────────────┘
```

Queste discontinuità si traducono in **derivata discontinua** → **gradiente che "salta"** → **velocità repulsive jerkose**.

Questo peggiorerebbe il problema di smoothing che hai già identificato!

---

## 5. Impatto sull'Implementazione Attuale

### 5.1. Componenti da Modificare

Se volessi implementare multi-resolution:

| File | Modifica | Effort |
|------|----------|--------|
| `voxel_grid_3d.hpp/cpp` | Cambiare completamente struttura dati | 🔴 Alto |
| `edt_calculator.cpp` | Riscrivere EDT per multi-res | 🔴 Molto Alto |
| `map3d_manager.cpp` | Gestire più livelli | 🟠 Medio |
| `voxelizer_sphere.cpp` | Voxelizzare su multi-res | 🟠 Medio |
| `getDistanceInterpolated()` | Interpolazione cross-level | 🔴 Alto |
| `getGradientInterpolated()` | Nuova logica gradiente | 🔴 Alto |

### 5.2. Rischi per il Controller

| Rischio | Probabilità | Impatto | Mitigazione |
|---------|-------------|---------|-------------|
| Discontinuità gradiente | Alta | ❌ Critico | Filtering pesante |
| Performance query | Media | ⚠️ Alto | Caching |
| Bug EDT | Alta | ❌ Critico | Test estensivi |
| Latenza update | Media | ⚠️ Medio | Parallelizzazione |

---

## 6. Confronto Quantitativo

### 6.1. Scenario: Raddoppiare l'Area Coperta

**Obiettivo**: Passare da 3×3×2 m a 6×6×4 m

#### Opzione A: Mantenere Risoluzione Uniforme 5cm

```
Prima:  60 × 60 × 40 = 144,000 voxel
Dopo:   120 × 120 × 80 = 1,152,000 voxel (8× !)

Memoria distanze: 4.6 MB per buffer
EDT tempo: ~100-200 ms (troppo per 20 Hz!)
```

❌ **Non praticabile** - troppo lento

#### Opzione B: Ridurre Risoluzione a 10cm

```
Dopo:   60 × 60 × 40 = 144,000 voxel (come prima!)

Ma: Precisione dimezzata vicino al robot
    Ostacoli piccoli (<10cm) potrebbero non essere rilevati
```

⚠️ **Compromesso** - peggiora la sicurezza vicino al robot

#### Opzione C: Multi-Resolution (2 livelli)

```
Livello 0 (vicino, 0-1.5m): 5cm → 60×60×40 = 144,000 voxel
Livello 1 (lontano, 1.5-3m): 10cm → 60×60×40 = 144,000 voxel
                                    ─────────────────────
                             Totale: 288,000 voxel (2×)
Con overlap gestione: +20%  ≈ 350,000 voxel

Area coperta: 6×6×4 m (quasi raddoppiata)
```

✅ **Buon trade-off** in teoria, MA:
- Implementazione complessa
- Rischio discontinuità

#### Opzione D: Griglia Uniforme con Risoluzione Adattiva "Smart"

```
Idea: 7cm uniforme

Dopo:   86 × 86 × 57 = 421,000 voxel (~3×)

Area: 6×6×4 m
Precisione: 7cm (accettabile)
EDT: ~50-80 ms (ok per 10-15 Hz)
```

✅ **Compromesso semplice** - nessuna modifica architetturale

### 6.2. Tabella Riassuntiva

| Opzione | Area | Voxel | EDT Time | Complessità | Rischio |
|---------|------|-------|----------|-------------|---------|
| A (5cm, 6m) | 6×6×4 | 1.15M | ~150ms ❌ | Bassa | Basso |
| B (10cm, 6m) | 6×6×4 | 144K | ~15ms ✅ | Bassa | ⚠️ Sicurezza |
| C (Multi-res) | 6×6×4 | ~350K | ~40ms | **Alta** | ⚠️ Discontinuità |
| D (7cm, 6m) | 6×6×4 | 420K | ~50ms ✅ | Bassa | Basso |

---

## 7. Alternative da Considerare

### 7.1. Approccio Ibrido: "Zone of Interest" Dinamica

Invece di cambiare la struttura della mappa, adattare il **centro della mappa** alla zona di interesse attuale:

```
┌─────────────────────────────────────────────────────────────────────┐
│                    MAPPA CENTRATA SU TCP/POI                        │
│                                                                     │
│    Invece di centrare la mappa su base_link,                        │
│    centrarla sulla "zona di lavoro attuale"                         │
│                                                                     │
│    ┌─────────────────────────────────────────────────────────┐     │
│    │                                                         │     │
│    │        [base_link]                                      │     │
│    │            │                                            │     │
│    │            ├────────●  ← TCP attuale                    │     │
│    │                     │                                   │     │
│    │               ┌─────┼────────┐                          │     │
│    │               │     │  MAPPA │                          │     │
│    │               │     ●        │  ← Centro mappa dinamico │     │
│    │               │              │                          │     │
│    │               └──────────────┘                          │     │
│    │                                                         │     │
│    └─────────────────────────────────────────────────────────┘     │
│                                                                     │
│    La mappa "segue" il manipolatore, massimizzando                 │
│    la copertura dove serve davvero.                                 │
└─────────────────────────────────────────────────────────────────────┘
```

**Pro:**
- Nessuna modifica all'EDT
- Implementazione semplice
- Alta risoluzione dove serve

**Contro:**
- Richiede aggiornamenti più frequenti se il TCP si muove velocemente
- Gestione frame TF più complessa

### 7.2. Parametri Adattivi Runtime

Cambiare risoluzione/dimensione a runtime in base alla situazione:

```yaml
# Modo "normale" - mappa compatta, alta risoluzione
mode_precise:
  size: 3.0m
  resolution: 0.04m

# Modo "esplorazione" - mappa estesa, risoluzione ridotta  
mode_exploration:
  size: 5.0m
  resolution: 0.08m
```

Il controller può switchare tra modi basandosi su:
- Velocità del robot
- Fase del task
- Densità ostacoli rilevata

### 7.3. Due Mappe Parallele

Usare **due mappe indipendenti**:

1. **Near-field map**: 2×2×2 m, 3cm resolution → Controllo reattivo
2. **Far-field map**: 6×6×4 m, 10cm resolution → Warning anticipato

```cpp
// Query combinata
RepulsionData getRepulsion(const Eigen::Vector3d& poi) {
    auto near = near_map_.queryWorld(poi);  // Priorità per repulsione
    auto far = far_map_.queryWorld(poi);    // Solo per warning/anticipazione
    
    if (near.valid) {
        return computeRepulsion(near);
    } else if (far.valid) {
        return computeWarningRepulsion(far);  // Più soft
    }
}
```

**Pro:**
- Nessuna discontinuità (mappe separate)
- Ogni mappa ottimizzata per il suo scopo
- EDT uniforme su entrambe

**Contro:**
- Doppio costo memoria e CPU
- Logica di fusione da implementare

---

## 8. Raccomandazione Finale

### 8.1. Risposta Breve

> **È una buona idea? Tecnicamente NO, nel tuo caso specifico.**

### 8.2. Motivazioni

1. **L'EDT richiede griglia uniforme**
   - L'algoritmo F&H che usi è estremamente efficiente (O(n)) ma assume spaziatura costante
   - Adattarlo a multi-resolution richiederebbe riscriverlo completamente

2. **Rischio discontinuità sul gradiente**
   - Stai già lavorando al problema delle velocità repulsive discontinue (vedi `Smoothing_Repulsive_Velocity.md`)
   - Introdurre una mappa multi-resolution **peggiorerebbe** questo problema

3. **Complessità implementativa elevata**
   - Dovresti modificare quasi tutti i componenti di `map3d`
   - Il debugging sarebbe significativamente più difficile

4. **Alternative più semplici esistono**
   - Ridurre leggermente la risoluzione (5cm → 7cm) triplica l'area con complessità zero
   - Due mappe parallele (near/far) mantengono l'architettura attuale

### 8.3. Cosa Ti Consiglio Invece

**📌 Opzione Raccomandata: Risoluzione Adattiva Semplice**

```yaml
# Parametri suggeriti per espandere la mappa
map3d:
  size_x: 4.0          # Da 3.0m a 4.0m (+33%)
  size_y: 4.0          # Da 3.0m a 4.0m (+33%)
  size_z: 2.5          # Da 2.0m a 2.5m (+25%)
  resolution: 0.06     # Da 0.05m a 0.06m (+20%)
  
  # Risultato:
  # Voxel: 67×67×42 = ~188,000 (solo +30% rispetto a 144,000)
  # Area coperta: +77% in volume
  # EDT: ~20-25ms (ok per 15-20 Hz)
```

Questo ti dà **molto più spazio** con **minimo impatto** su:
- Tempo EDT
- Complessità codice
- Smoothness del gradiente

### 8.4. Se in Futuro Servisse Davvero Multi-Resolution

Se nel futuro (es. con base mobile) avrai davvero bisogno di mappe estese:

1. **Considera Voxblox** ([github.com/ethz-asl/voxblox](https://github.com/ethz-asl/voxblox))
   - Già implementa ESDF incrementale
   - Supporta mappe molto grandi
   - Usato in produzione su droni e robot mobili

2. **O implementa due mappe parallele** come descritto in §7.3

---

## Appendice: Riferimenti Tecnici

### A.1. Euclidean Distance Transform

> Felzenszwalb, P. F., & Huttenlocher, D. P. (2012). 
> *Distance transforms of sampled functions.*
> Theory of Computing, 8(1), 415-428.

L'algoritmo assume esplicitamente:
- Griglia cartesiana regolare
- Spaziatura uniforme lungo ogni asse

### A.2. Multi-Resolution ESDF

Approcci esistenti in letteratura:
- **Wavemap** (ETH Zurich) - Octree + approssimazione ESDF
- **VDB-ESDF** - OpenVDB per mappe sparse

Nessuno offre la stessa efficienza dell'F&H uniforme per mappe dense.

---

*Fine documento*
