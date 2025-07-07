# Furkan Kalay 150220049

import os
from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
from sklearn.metrics import confusion_matrix

def vectorizeImages(root, size=(92, 112)):
    xArray, yArray = [], []
    for subject in sorted(os.listdir(root)):
        subjectPath = os.path.join(root, subject)
        if not os.path.isdir(subjectPath):
            continue
        label = int(subject.lstrip('s'))  # klasör adı 's1' → 1
        for fname in sorted(os.listdir(subjectPath)):
            if not fname.lower().endswith('.pgm'):
                continue
            # 1. Görüntüyü aç ve gri tonlamaya çevir
            img = Image.open(os.path.join(subjectPath, fname)).convert('L')
            # # 2. Yeniden boyutlandır (opsiyonel, ORL zaten 92×112)
            # img = img.resize(size, Image.ANTIALIAS)
            # 3. NumPy array'e çevir
            arr = np.asarray(img, dtype=np.float32)  # shape: (112, 92)
            # 4. Flatten: tek boyutlu vektör (112*92 = 10304 uzunluk)
                              # shape: (10304,)
            xArray.append(arr.flatten())
            yArray.append(label)

    # Tüm vektörleri bir matris hâline getir
    x = np.stack(xArray, axis=0)  # shape: (N, 10304)
    y = np.array(yArray, dtype=np.int32)
    return x, y

# 1. Data matrix’i oluşturduktan sonra
x, y = vectorizeImages('archive', size=(92,112))
# 2. Ortalama yüz
meanFace = np.mean(x, axis=0)
# 3. Normalize et (center)
X_centered = x - meanFace

#task1 experiment
# --- 3. 10 örnekten oluşan bir altküme seç (ör. s1–s10 klasörlerinin ilk resmi) ---
subset_vecs = []
for subject in range(1, 11):  # 1'den 10'a kadar
    img_path = os.path.join('archive', f's{subject}', '1.pgm')
    img = Image.open(img_path).convert('L') #.resize((92,112), Image.ANTIALIAS)
    vec = np.asarray(img, dtype=np.float32).flatten()
    subset_vecs.append(vec)
X_sub = np.stack(subset_vecs, axis=0)  # shape = (10, P)

# --- 4. Altkümeden ortalama yüz ---
mean_sub = np.mean(X_sub, axis=0)

# --- 5. Görselleştirme ---
h, w = 112, 92  # yükseklik, genişlik
fig, axes = plt.subplots(1, 3, figsize=(12, 4))

axes[0].imshow(meanFace.reshape(h, w), cmap='gray')
axes[0].set_title(f'Mean Face – All ({x.shape[0]} img)')
axes[0].axis('off')

axes[1].imshow(mean_sub.reshape(h, w), cmap='gray')
axes[1].set_title('Mean Face – Subset (10 img)')
axes[1].axis('off')

# Fark haritası (mutlak farkları gösterir)
diff = np.abs(meanFace - mean_sub)
axes[2].imshow(diff.reshape(h, w), cmap='hot')
axes[2].set_title('Absolute Difference')
axes[2].axis('off')

plt.tight_layout()
plt.show()

# --- 6. Niceliksel Karşılaştırma (MSE) ---
mse = np.mean((meanFace - mean_sub)**2)
print(f"Mean Face MSE (all vs subset): {mse:.2f}")


#task 2
def compute_eigenfaces(X_centered): #with PCA
    """
    X_centered: (N, P) boyutlu, ortalaması çıkarılmış veri matrisi.
    Returns:
      eigenfaces: (K, P) boyutlu; her satır bir eigenface (principal component)
      eigenvalues: (K,) boyutlu; her component’ın varyansı
    """
    N, P = X_centered.shape

    # Küçük kovaryans matrisi: C_small = X X^T / (N-1), boyutu (N,N)
    C_small = (X_centered @ X_centered.T) / (N - 1)

    # 1. Küçük matrisin öz-değerlerini & öz-vektörlerini al
    eigvals_small, eigvecs_small = np.linalg.eig(C_small)

    # 2. Büyük uzaydaki gerçek eigenface’leri hesapla
    #    v_small: (N,) → u = X^T @ v_small  → (P,)
    #    Sonra normalize et
    idx = np.argsort(eigvals_small)[::-1]       # en büyükten en küçüğe sıralama
    eigvals = eigvals_small[idx]
    eigenfaces = []
    for i in idx:
        v = eigvecs_small[:, i]                  # küçük uzay vektörü
        u = X_centered.T @ v                      # büyük uzaya taşı
        u /= np.linalg.norm(u)                   # normalize
        eigenfaces.append(u)
    eigenfaces = np.stack(eigenfaces, axis=0)   # şekil: (N, P)

    return eigenfaces, eigvals


# --- 2. İlk 10 Eigenface’i Görselleştirme ---
def show_top_eigenfaces(eigenfaces, h, w, M=10, cols=5):
    """
    Top M eigenface’i h×w olarak gösterir.
    cols: her satırda kaç tane resim.
    """
    rows = int(np.ceil(M / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(2.5*cols,2.5*rows))
    for i, ax in enumerate(axes.flat):
        if i < M:
            img = eigenfaces[i].reshape(h, w)
            ax.imshow(img, cmap='gray')
            ax.set_title(f'EF {i+1}')
        ax.axis('off')
    plt.tight_layout()
    plt.show()


# --- 3. Eigenvalue Spectrum Plot’u ---
def plot_eigenvalue_spectrum(eigvals):
    """
    Bileşen indeksine karşı özdeğer (varyans) grafiği.
    """
    plt.figure(figsize=(6,4))
    plt.plot(np.arange(1, len(eigvals)+1), eigvals, marker='o')
    plt.xlabel('Component index')
    plt.ylabel('Eigenvalue (variance)')
    plt.title('Eigenvalue Spectrum')
    plt.grid(True)
    plt.show()


# --- 4. Kümülatif Varyans Grafiği ---
def plot_cumulative_variance(eigvals):
    """
    M komponent seçildiğinde ne kadar toplam varyans yakalanır.
    """
    cumvar = np.cumsum(eigvals) / np.sum(eigvals)
    plt.figure(figsize=(6,4))
    plt.plot(np.arange(1, len(cumvar)+1), cumvar, marker='o')
    plt.xlabel('Number of components (M)')
    plt.ylabel('Cumulative explained variance')
    plt.title('Cumulative Variance Explained')
    plt.grid(True)
    plt.show()


# --- 5. Farklı M Değerleri İçin Deney ---
def experiment_different_M(eigenfaces, eigvals, h, w, Ms=[10,20,50,100,200,300], cols=5):
    """
    Her bir M için:
      - İlk M eigenface’i göster
      - (isteğe bağlı) grafikleri kaydetmek yerine gösteriyoruz
    """
    print(">> Top 10 Eigenfaces")
    show_top_eigenfaces(eigenfaces, h, w, M=10, cols=cols)

    print(">> Eigenvalue Spectrum")
    plot_eigenvalue_spectrum(eigvals)

    print(">> Cumulative Variance")
    plot_cumulative_variance(eigvals)

    for M in Ms:
        print(f">> Eigenfaces for M = {M}")
        show_top_eigenfaces(eigenfaces, h, w, M=M, cols=cols)

eigenfaces, eigvals = compute_eigenfaces(X_centered)

# 3) Task2 adımları
show_top_eigenfaces(eigenfaces, h, w, M=10)              # ilk 10
plot_eigenvalue_spectrum(eigvals)                       # değer spektrumu
plot_cumulative_variance(eigvals)                       # kümülatif varyans

# 4) Deney: farklı M’ler için
experiment_different_M(eigenfaces, eigvals, h, w)

#task 3
def reconstruct_face(x, mean_face, eigenfaces, M):
    x_centered = x - mean_face
    U = eigenfaces[:M]             # (M, P)
    coeffs = U @ x_centered        # (M,)
    x_recon = mean_face + coeffs @ U  # (P,)
    return x_recon

def compute_mse(x, x_recon):
    return np.mean((x - x_recon)**2)

def find_min_M(x, mean_face, eigenfaces, threshold=500):
    for M in range(1, eigenfaces.shape[0] + 1):
        x_recon = reconstruct_face(x, mean_face, eigenfaces, M)
        mse = compute_mse(x, x_recon)
        if mse < threshold:
            return M, mse
    return None, None

def plot_reconstructions(X, mean_face, eigenfaces, indices, Ms, h, w):
    for idx in indices:
        x = X[idx]
        fig, axes = plt.subplots(1, len(Ms)+1, figsize=(2.5*(len(Ms)+1), 3))
        axes[0].imshow(x.reshape(h, w), cmap='gray')
        axes[0].set_title(f'Original idx={idx}')
        axes[0].axis('off')
        for j, M in enumerate(Ms):
            x_recon = reconstruct_face(x, mean_face, eigenfaces, M)
            mse = compute_mse(x, x_recon)
            axes[j+1].imshow(x_recon.reshape(h, w), cmap='gray')
            axes[j+1].set_title(f'M={M}\nMSE={mse:.1f}')
            axes[j+1].axis('off')
        plt.tight_layout(); plt.show()

#task 3 experiment
idx10 = np.where(y == 10)[0][0]
idx11 = np.where(y == 11)[0][0]
Ms = [10, 20, 50, 100, 200, 300]
plot_reconstructions(x, meanFace, eigenfaces, [idx10, idx11], Ms, h, w)

# MSE<500 sağlayan minimum M'i bul
for idx in [idx10, idx11]:
    Mmin, mse_val = find_min_M(x[idx], meanFace, eigenfaces, threshold=500)
    print(f'Face idx={idx}: min M for MSE<500 = {Mmin}, MSE={mse_val:.2f}')

#task 4

def project(X_centered, eigenfaces, M):
    return X_centered @ eigenfaces[:M].T


def train_test_split_one_per_subject(y):
    train_idx, test_idx = [], []
    for lab in np.unique(y):
        idxs = np.where(y==lab)[0]; idxs.sort()
        test_idx.append(idxs[0]); train_idx.extend(idxs[1:])
    return np.array(train_idx), np.array(test_idx)


def classify_1nn(train_proj, train_labels, test_proj):
    preds=[]
    for v in test_proj:
        d = np.linalg.norm(train_proj - v, axis=1)
        preds.append(train_labels[np.argmin(d)])
    return np.array(preds)


def evaluate_classification(X_centered, y, eigenfaces, M):
    train_idx, test_idx = train_test_split_one_per_subject(y)
    proj = project(X_centered, eigenfaces, M)
    train_p, train_l = proj[train_idx], y[train_idx]
    test_p, test_l = proj[test_idx], y[test_idx]
    preds = classify_1nn(train_p, train_l, test_p)
    acc = np.mean(preds==test_l)
    cm = confusion_matrix(test_l, preds)  # burada kullanılır
    return acc, cm

# Confusion matrix ve accuracy plot fonksiyonları
def plot_confusion_matrix(cm, classes, title='Confusion matrix'):
    plt.figure(figsize=(8,6)); plt.imshow(cm, cmap=plt.cm.Blues)
    plt.title(title); plt.colorbar()
    ticks = np.arange(len(classes));
    plt.xticks(ticks, classes, rotation=90); plt.yticks(ticks, classes)
    plt.ylabel('True label'); plt.xlabel('Predicted label'); plt.tight_layout(); plt.show()


def plot_accuracy_vs_M(Ms, accs):
    plt.figure(); plt.plot(Ms, accs, 'o-')
    plt.xlabel('M'); plt.ylabel('Accuracy')
    plt.title('Accuracy vs. M'); plt.grid(True); plt.show()

#task 4 experiment

Ms = [10,20,50,100,200,300]
accs = {}
cms = {}
for M in Ms:
    acc, cm = evaluate_classification(X_centered, y, eigenfaces, M)
    accs[M] = acc
    cms[M] = cm
    print(f'M={M}, acc={acc:.3f}')
    plot_confusion_matrix(cm,classes=[str(c) for c in np.unique(y)],title=f'CM M={M}')
plot_accuracy_vs_M(Ms, [accs[M] for M in Ms])

# ----------------------
# Task 5: Robustness to Noise (from scratch)
# ----------------------
def add_gaussian_noise(arr, sigma):
    """Adds Gaussian noise: arr normalized 0–255, sigma fraction."""
    noisy = arr + np.random.normal(0, sigma*255, arr.shape)
    return np.clip(noisy, 0, 255)

def add_salt_pepper_noise(arr, amount):
    """Adds salt & pepper noise: amount fraction of pixels."""
    out = arr.copy().flatten()
    P = out.size
    num_sp = int(amount * P)
    coords = np.random.choice(P, num_sp, replace=False)
    half = num_sp // 2
    out[coords[:half]] = 0
    out[coords[half:]] = 255
    return out.reshape(arr.shape)

def save_noisy_samples(root, out_dir, subjects, noise_levels):
    """Saves noisy images for each subject and noise level."""
    os.makedirs(out_dir, exist_ok=True)
    for sigma in noise_levels:
        for subj in subjects:
            img = Image.open(f"{root}/s{subj}/1.pgm").convert('L')
            arr = np.asarray(img, dtype=np.float32)
            noisy = add_gaussian_noise(arr, sigma)
            plt.imsave(f"{out_dir}/noisy_{sigma:.1f}_{subj}.png", noisy, cmap='gray')


def recognition_under_noise(X, y, mean_face, eigenfaces, M, noise_levels, noise_type='gaussian'):
    """Returns list of accuracies for each noise level."""
    train_idx, test_idx = train_test_split_one_per_subject(y)
    train_proj = project(X[train_idx] - mean_face, eigenfaces, M)
    accuracies = []
    for lvl in noise_levels:
        # add noise to all test images
        X_test = X[test_idx].reshape(-1, mean_face.size)
        if noise_type == 'gaussian':
            noisy = np.array([add_gaussian_noise(x.reshape(mean_face.shape), lvl).flatten() for x in X_test])
        else:
            noisy = np.array([add_salt_pepper_noise(x.reshape(mean_face.shape), lvl).flatten() for x in X_test])
        test_proj = project(noisy - mean_face, eigenfaces, M)
        preds = classify_1nn(train_proj, y[train_idx], test_proj)
        accuracies.append(np.mean(preds == y[test_idx]))
    return accuracies

def plot_accuracy_vs_noise(noise_levels, accs, title, out_path=None):
    """Plots and optionally saves accuracy vs noise level."""
    plt.figure(); plt.plot(noise_levels, accs, 'o-')
    plt.xlabel('Noise level'); plt.ylabel('Recognition accuracy')
    plt.title(title); plt.grid(True)
    if out_path:
        plt.savefig(out_path)
    plt.show()

def task5_experiment(root='archive', eigenfaces=None, mean_face=None, X=None, y=None,
                     M=50, subjects=None, noise_levels=None, out_dir='noise'):
    """
    Runs Task5: saves noisy samples, evaluates recognition, and saves accuracy plot.
    """
    if subjects is None:
        subjects = list(range(1, 11))
    if noise_levels is None:
        noise_levels = [0.1,0.2,0.3,0.4,0.5]
    # Save sample images
    save_noisy_samples(root, out_dir, subjects, noise_levels)
    # Evaluate Gaussian
    gauss_acc = recognition_under_noise(X, y, mean_face, eigenfaces, M, noise_levels, 'gaussian')
    # Evaluate Salt & Pepper
    sp_acc = recognition_under_noise(X, y, mean_face, eigenfaces, M, noise_levels, 'salt_pepper')
    # Plot and save accuracy vs noise
    plot_accuracy_vs_noise(noise_levels, gauss_acc,
                           f'Accuracy vs Gaussian Noise (M={M})',
                           out_path=f'{out_dir}/accuracy_vs_noise_gaussian.png')
    plot_accuracy_vs_noise(noise_levels, sp_acc,
                           f'Accuracy vs Salt & Pepper Noise (M={M})',
                           out_path=f'{out_dir}/accuracy_vs_noise_sp.png')

def display_noisy_samples(root, subjects, noise_levels, h, w):
        """Shows original and Gaussian-noisy images for each subject."""
        fig, axes = plt.subplots(len(subjects), len(noise_levels)+1,
                                 figsize=(2*(len(noise_levels)+1), 2*len(subjects)))
        for i, subj in enumerate(subjects):
            # Original
            orig = Image.open(f"{root}/s{subj}/1.pgm").convert('L')
            axes[i, 0].imshow(orig, cmap='gray')
            axes[i, 0].set_title(f's{subj} orig')
            axes[i, 0].axis('off')
            # Noisy versions
            arr = np.asarray(orig, dtype=np.float32)
            for j, lvl in enumerate(noise_levels):
                noisy = add_gaussian_noise(arr, lvl)
                axes[i, j+1].imshow(noisy.reshape(h, w), cmap='gray')
                axes[i, j+1].set_title(f'{lvl:.1f}')
                axes[i, j+1].axis('off')
        plt.suptitle('Original and Noisy Samples (Gaussian)')
        plt.tight_layout(rect=[0,0,1,0.95]); plt.show()

#task 5 experiment

task5_experiment(
        root='archive',
        eigenfaces=eigenfaces,
        mean_face=meanFace,
        X=x,
        y=y,
        M=50,
        subjects=list(range(1, 11)),       # subjects s1..s10
        noise_levels=[0.1, 0.2, 0.3, 0.4, 0.5],
        out_dir='noise'
    )

display_noisy_samples('archive', list(range(1,11)), [0.1,0.2,0.3,0.4,0.5], h, w)

# 5. Print recognition accuracies for Gaussian noise
gauss_acc = recognition_under_noise(x, y, meanFace, eigenfaces, 50,
                                       [0.1,0.2,0.3,0.4,0.5], 'gaussian')
print('Recognition accuracy under Gaussian noise (M=50):')
for lvl, acc in zip([0.1,0.2,0.3,0.4,0.5], gauss_acc):
    print(f'  noise={lvl:.1f} -> accuracy={acc:.3f}')
# 6. Plot accuracy vs Gaussian noise again for clarity
plot_accuracy_vs_noise([0.1,0.2,0.3,0.4,0.5], gauss_acc,'Accuracy vs Gaussian Noise (M=50)')

