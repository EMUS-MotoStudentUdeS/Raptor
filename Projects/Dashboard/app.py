import json
import sys
import threading
import time
from datetime import datetime

import serial
import serial.tools.list_ports
from flask import Flask, jsonify, render_template_string

BAUD_RATE = 115200
PORT_PAR_DEFAUT = "COM8"

# Conversion valeur brute -> valeur physique, (scale, offset, unite), tirees
# du DBC reel. Le firmware envoie toujours la valeur BRUTE (avant scale) ;
# valeur_physique = brut * scale + offset. Signaux absents d'ici (flags,
# BMS, signaux dont le scale du DBC vaut deja 1/0) restent affiches bruts.
CONVERSIONS = {
    "v_bat":         (0.1, 0, "V"),
    "i_bat":         (0.1, 0, "A"),
    "i_motor":       (0.1, 0, "A"),
    "iq_ref":        (0.1, 0, "A"),
    "key_sw_volt":   (0.01, 0, "V"),
    "motor_temp":    (1, -40, "C"),
    "inverter_temp": (1, -40, "C"),
    "throttle_req":  (0.784314, 0.335, "%"),
    "torque":        (0.784314, 0.335, "%"),
    "odometer_lo":   (0.01, 0, "km"),
    "odometer_hi":   (0.01, 0, "km"),
    "fault_code":    (0.588235, 0, ""),
    "fault_level":   (0.0156863, 0, ""),
    "current_sensor": (4.65661e-07, 0, "A"),
    "flow_drive":    (0.000915527, 0, "L/min"),
    "flow_motor":    (0.000915527, 0, "L/min"),
}

app = Flask(__name__)

etat = {
    "donnees": {},
    "derniere_maj": None,
    "connecte": False,
    "erreur": None,
    "port": None,
}
verrou = threading.Lock()


def lire_serie(port):
    while True:
        try:
            with serial.Serial(port, BAUD_RATE, timeout=1) as ser:
                with verrou:
                    etat["connecte"] = True
                    etat["erreur"] = None
                    etat["port"] = port
                print(f"Connecte a {port}")
                while True:
                    ligne = ser.readline().decode("utf-8", errors="ignore").strip()
                    if not ligne.startswith("{"):
                        continue
                    try:
                        donnees = json.loads(ligne)
                    except json.JSONDecodeError:
                        continue
                    with verrou:
                        etat["donnees"] = donnees
                        etat["derniere_maj"] = datetime.now().strftime("%H:%M:%S")
        except serial.SerialException as e:
            with verrou:
                etat["connecte"] = False
                etat["erreur"] = str(e)
            time.sleep(2)


@app.route("/")
def index():
    return render_template_string(PAGE_HTML)


@app.route("/api/data")
def api_data():
    with verrou:
        return jsonify(etat)


@app.route("/api/conversions")
def api_conversions():
    return jsonify(CONVERSIONS)


PAGE_HTML = """
<!doctype html>
<html lang="fr">
<head>
<meta charset="utf-8">
<title>Raptor - Telemetrie</title>
<style>
  body { font-family: monospace; background: #111; color: #eee; margin: 0; padding: 1.5rem; }
  .entete { display: flex; align-items: center; gap: 0.75rem; margin-bottom: 0.25rem; }
  .entete img { height: 2.5rem; width: auto; display: block; }
  h1 { font-size: 1.2rem; margin: 0; }
  #statut { margin-bottom: 1rem; font-size: 0.9rem; }
  .dot { display: inline-block; width: 0.6rem; height: 0.6rem; border-radius: 50%; margin-right: 0.4rem; }
  .ok { background: #3c9; }
  .ko { background: #e55; }
  input#filtre { background: #222; color: #eee; border: 1px solid #444; padding: 0.3rem 0.5rem; margin-bottom: 0.75rem; width: 16rem; }
  table { border-collapse: collapse; width: 100%; max-width: 40rem; }
  td, th { border: 1px solid #333; padding: 0.3rem 0.6rem; text-align: left; }
  th { background: #1a1a1a; }
  tr:nth-child(even) { background: #181818; }

  .graphiques { display: flex; gap: 1rem; flex-wrap: wrap; margin-bottom: 1.25rem; }
  .carte-graph { background: #1a1a19; border: 1px solid #2c2c2a; border-radius: 6px;
                 padding: 0.75rem 1rem; flex: 1 1 320px; min-width: 300px; position: relative; }
  .carte-graph h2 { font-size: 0.85rem; font-weight: 600; margin: 0 0 0.5rem 0; color: #eee; }
  .carte-graph canvas { width: 100%; height: 160px; display: block; cursor: crosshair; }
  .legende-graph { display: flex; gap: 1rem; margin-top: 0.4rem; font-size: 0.78rem; color: #c3c2b7; flex-wrap: wrap; }
  .legende-graph .cle { display: inline-flex; align-items: center; gap: 0.35rem; }
  .legende-graph .trait { display: inline-block; width: 14px; height: 2px; border-radius: 1px; }
  .tooltip-graph { position: absolute; pointer-events: none; background: #0d0d0d;
                   border: 1px solid #383835; border-radius: 4px; padding: 0.4rem 0.6rem;
                   font-size: 0.75rem; color: #eee; display: none; white-space: nowrap; z-index: 10; }

  /* Surlignage bref quand une valeur change - vaut pour TOUT capteur, meme
     sans graphique dedie (voir cellesChangees dans maj()). */
  tr.changee { animation: flashChangement 700ms ease-out; }
  @keyframes flashChangement {
    0%   { background: rgba(57, 135, 229, 0.45); }
    100% { background: transparent; }
  }
</style>
</head>
<body>
  <div class="entete">
    <img src="data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAYgAAABqCAYAAACmuMYbAAAAAXNSR0IArs4c6QAAAARnQU1BAACxjwv8YQUAAAAJcEhZcwAAHYcAAB2HAY/l8WUAACIZSURBVHhe7d15dFNl/j/w981N0jRN940uQPfSsorsIKIdoAqoKC58VcARdNRRR5nx+3U44/Lzhz8Zl0EPR/yCYhFwxvGIoLKICLiwjWxlb6GlG91LKW3aZrv390dLxz7cJDdPUmiYz+uczDnzeWJJ0+T53Pssn0cAIIMQQghhaNgAIYQQAkoQhBBCnKEEQQghRBElCEIIIYooQRBCCFFECYIQQogiShCEEEIUUYIghBCiiBIEIYQQRZQgCCGEKKIEQQghRBElCEIIIYooQRBCCFFECYIQQogiShCEEEIUUYIghBCiiBIEIYQQRcJ/4olygiBA1ArQ6jQQNIDDJsNqcbBPc0mjEaA3iBCuUoq1WSTYbRIb7iYoKAh6vZ4Nq2Kz2WA2myHLV34cAgMDYTAY2LAqsizDbrfDarXCZrMp/nx3tFotTCYTBEFgm1SRZRk2mw1WqxV2u53rNbBCQ0Oh0fD98VtbW2GxWNiw10RRhNFohFarZZvckmUZ7e3taG9vZ5tUE0URer0egYGBXK/hapAkCU1NTbDZbGwTUfAflSB0ASL6Z4Yg68ZIGEP00Bs00GgE7FpfitKCS+zTXUpIDcaE6YkIDOr5L4Kl3YH931ai+MRFtqmLKIp46aWXkJmZyTap8u2332LNmjWw2+3d4lFRUXjiiScwcODAbnG1LnfOra2tKC8vx4YNG3D69Gk4HOoSsiiKmDt3LiZPnsydICRJgtVqRUtLC44dO4aNGzeiurqafZpqJpMJb731FsLCwtgmVZYuXYp9+/axYa8NGDAATzzxBGJjY9kmt1pbW/HJJ59g165dbJNbwcHBmDRpEsaPH4/o6GgEBwdDp9OxT+sVmpqa8MILL6C2tpZtIgr+IxKEIABJWWHIfTgV8UlBMIXqoRE7Opv6yla8+dQ+tLeq67AAQKvTYMrsZEyenQyNhq/T8kTd+Vase+sEzp10niAiIyOxZ88eZGRksE1uORwOLFiwAKtXr4Yk/fsuRRAELFq0CC+++CKMRmO3/4aHxWJBZWUl8vLy8Pbbb8NsNrNP6UYQBMyYMQPvv/8+EhIS2GaPybKMlpYWVFVVYcmSJcjLy+v2+6qVm5uLzz77DCEhIWyTWxaLBdOmTcP333/PNnnt7rvvxkcffcSVuIqLi/Hggw96lLiMRiMWLFiA+fPnIzY2FhERERBFkX1ar3Lo0CGMGzeuR+7grkd898h+JNCkw/RH0vDsOyMwYHgEQiICupKDJMnYvem8R8kBAEKjAjD2tsSrkhwAoKG6DZXnmtlwN1lZWVwdFgA0NzejoqKiW2cpiiIeeOABvPbaaz5JDgAQEBCA5ORkvPrqq1i7di2CgoLYp3STnZ2NJUuW+CQ5oDPhBAcHIyMjAytWrMDSpUsRGBjIPs2tKVOmcL8nJ06cQHl5ORv2miiK6Nu3L4KDg9kmVRoaGnDmzBk2rEgURYwZMwbbt2/H3/72NwwaNAjR0dG9PjkAwPbt2yk5eOC6TRCCAPTpH4SHXxiInPuSFYcnqkvNOL7fs1tNQSPg5pn9EBLBN9bvKVnuuIOwtDlPYhqNBjfffDPCw8PZJlUKCgpQUVHR9f91Oh3uvfde5OXldXueL91111145513nM6Z9OvXD8uWLcOAAQPYJp+4PHQ1d+5cj8bLAwICEB8fz90ZHjhwAJWVlWzYayEhIZg0aRL36zp16hQuXnR+h3qZyWTCggUL8PXXX2Ps2LGK36veSpIkbNq0iQ0TF67bBBEWbcCdCzKQNSIKSp9hh03Csb21aKzxbFIuMSUYY6b65opWFVlG8fFGNtpNYGAg0tPTERAQwDa5Jcsy8vPzUVJSAnQmm9zcXCxZssRp5+0r9913H+644w42jOjoaCxbtgyTJk1im3wqJCQETz31lEd3KOnp6UhPT+fqGO12O8rLy9Ha2so2ec1kMmHEiBFsWLVNmza5nRcKCwvDK6+8gnfffRdRUVFsc69XU1ODgwcPsmHiwnWbIKbNTcWAGyO7hpNYjXXtOLm/Hjar+jFoUSvgtjkpCDDwXaXxsLQ7UHD4AhvuJjY2FklJSWxYFZvNhoqKCrS1tQGdwzpLly5F37592af6XGho6BWTz4GBgViyZAmmTZvW7bk9ZeDAgcjKymLDTmVnZ3PN8wBAbW0tDh06xDXv4U5iYiLX5PRle/fuZUPdGAwGLFy4EE899VSPXzj0lB9++MHtvBfp7rpMECNujcOInHiXcwRnjjSitNCzlUvpQyPQPzOUDfeo2vJWmC+5XpKXmZmJwYMHs2FVLl68iEOHDgEA+vTpg3Xr1iElJYXrCtlTgiAgJSUF8fHxQOfwzcKFCzFnzhzuJaSeEgQBjzzyCBtWpNFoEBUVxT3/UFVVhQMHDrBhn5g9ezb3yqGTJ0+iqqqKDXcRBAHTp0/HM888w73c+VqTZRnffvstGyZuXJ1v4VUUFmXAtHmpisNKl1naHPh5UzlkSf0CLmOwDiNy4hAUwvcl5HXk5xo21I0oikhJSeGef6iursbu3buRnJyM1atXY8iQIexTelTfvn3Rt29fCIKARx99FC+++CL3ODqvMWPGqEqIwcHBmDBhAnfyqq2tRWOj6+FCXlOnTmVDqq1ateqK5c2/lpCQgNdee417EURv0NjYSMNLHK67Za4T7+yHux7LgKh1/oXfs7kC/3zvFDzZLxUcpsegsdEIDr96t9eyDPxr23k0NVjZpi4GgwHLli3Do48+yjap8tVXX2HmzJm48cYbkZOTo7pzFgQB8fHxmDNnjtvVSK6UlJRg3rx52L9/Px599FGPlmiKoohRo0YhJyfHqyvburo69OnTx+3QT79+/bB//3706dOHbVLl6aefxrJly9iw1xISElBUVMQ9BzV48GCcOHGCbeqyYsUKzJ8/X1USVdLc3IzPP/8cR48edZmIetLFixexceNGtLS0sE3EhesqQegNIub8z2AMHhvNNnVpbbbh//52t9thG5YgdHSK4PuOcJMcrv88kZGROHLkCBITE9kmVZ588kksX74coih63AFoNBrccccdyMvL404S+fn5mDdvHvLz81Unp18LCAjA4sWL8cwzz3j8+i8rKipCenq62x3WgwYNwuHDhz1a9fRr/fv3R1lZGRv22p/+9Cf89a9/ZcOqlJeXIyMjw+kO6vT0dBw7dowr+aDzAiA3NxclJSU+28XOy90FALkS371yLxURY0B4tPMrSVmSsX9bpcfJAZ1X85IkQ3Jc3Yc7SUlJHq3C+bX29nasX78e6NwsZ7fbPXpYrVb89NNP2LZtG/ujVSspKUFZWVlXSQ5PH2azGYsWLfLqy//jjz+q6rjuuece7uTQ2Njo1e5tZwRBQG5uLhtW7ZtvvoHV6vwOdf78+dzJoaKiAnfeeScKCgpgsVjgcDggSdI1exDPXVcJIjrRiNAo5x/mC7XtOPyj6zF9f8OuAvJESUkJamq8ez9kWeb+8kmShJqaGq/H5Xn//cu++OILNqTo4YcfZkOqffLJJy47Yl5RUVHcK9gkScKBAwecJkej0Ygbb7yRDav22Wefqd58R3qn6yZBiFoBffqZEBSsPIksSTJOHWhATdn1s8xNp9N51Wl9+eWXbMhjiYmJGDp0KBtWxWKx4PTp0047KLXUTjIraWtrQ2lpKRu+QmhoKPr168eGVVu1ahUb8omJEyciMjKSDavS0NCAs2fPOn3/Bw4ciP79+7NhVcxmM/Lz850OXRH/cN0kiIBALeKSTE73PTQ3WnFifx3aW6/NJFlPiImJQXZ2NhtWxWazYc2aNWzYYwkJCUhJSWHDqjQ1NeGrr75iwx7zZlnsgQMH0NDQwIavcPfdd3MvIzWbzSgqKmLDPjFixAjuZbdHjhzp2iCpZNiwYYiJiWHDqpSUlODUqVNOkw/xD3zfql7IaNIiIdV5HZqSU004e9S7oYzeZuLEiWxIterqaq+HlwRBwIABA7g75wsXLqC4uJgNe0Sn03l1Zb9z505VCWLOnDlsSLWtW7d2bUT0peDgYCQlJXHNi8iyjKKiItTX17NNQOf7mpyczL34oLS0VNWdGend+L7ZvVBknBGRfZQLr9ntEnZvroC13XUpAX8iiiLuvvtuNqza7t27vS5aptVqcdddd7Fh1fbt2+f1FWZmZib3Va7VakVlZaXbswFMJhP3UAs6S6l7+3sqSUtLQ3Z2NtfwWltbGwoLC50OAcXExGDYsGFcK8skSUJ5ebnXc0vk2rtuEkTKwDDo9Mq/TsGhCzib77pchb8JDw/nLmRnt9uxY8cOr69qvan/I8syli9fzoY9Nm7cOCQnJ7NhVUpLS3HixAm3nff48eM92p/xa62trSgtLXX7b/Do168fd+Kqq6vD/v37nU7wR0dHc5cUkWUZFovF6c8m/kO5R/VDA4ZHsCEAgM0q4etVZ+Cw++4LajBqkTo4HKlDInrgEa5qM15WVhb37umWlpYrynvzGDp0KHddnra2Nq93tgqCgNjYWO5hkLNnz+LIkSNs+Ao5OTncZbSPHj2Kc+fOsWGviaKIpKQk7td18eJFFBQUsOEugYGB3ElRFEXccMMNSEtLg1arhSiKPfLQaDRcd09Evetio1xYVAAWrRoPfUD322FZBg7tqsbaN4+r2lOgRnCYHrN+PwBDJ/AXRnNFsktY8sQ+1JQ7X22l0Wjwwgsv4OWXX+baQfzLL79g3rx5OHnyJNukmiAI2LBhg2I1VjV27NiBnJwcNuyRqKgovP/++7j33nvZJrdkWcaaNWswb948l1f3er0eH330ER588EGuzmjFihV4/vnnfV4kLjQ0FOvWreMuavjpp59izpw5Tiu4jh8/Hps2bUJoKF/tMYfDgSNHjuC7777DpUue1TxTy2q1orq6GuXl5SguLkZlZaXXFz2ku+siQYz8TRwe+tMgNozmRivWvX0Cp35RnojzVFCIDtN/m46xuQkuaz15o76yDYvn73aZ0EwmE9577z3MmzfP405LlmXk5eXh97//vVdlp4OCglBbW8u1gkaWZTz44IP4+9//zjZ5ZOjQodi2bRvXHITFYsFjjz2GTz75hG3qJjs7Gx9//DFGjRrFNrlls9nw2muvYfHixT7vuOLi4nDw4EHExcWxTao88MAD+Oyzz9hwlzFjxuDrr7/u1WW95c7NlfX19Th8+DC+/PJLrF271um8CvGc3w8xCQKUS2vIQNGxRpR7WLHVGX2AiIl39cOIW/v0WHIAgNOHG1wmB3ROIPJWXLXZbCgtLfUqOQBARkYGV3JA55Wft8NL6DyfIDpa4W+vgsViwebNm9nwFTIzM7nnempra12O83sjNTWV+3eXZRl79uxhw920tLSgrq6ODfcqgiBAp9MhLi4Ot912G95++22sXLmS+66HXMnvE4QxWIekrCvHSltbbDi+rw4tTd7vXtVoBAydEIOJd/S9YhjLlyQHsHfLeTZ8hdTUVO7y3s3Nzfjll1/YsMfuv/9+NqRaYWEhmptdH6Gqxq233sqVJNG5xPbCBdcLFzQaDaKjo2EymdgmVerq6nqsvPecOXO4lrei8/Q4V+W9AaCyshIHDx50OfzWmwiCgJCQEMyePRuvv/4697wU6c7vE0RiWggMxiu/KA3V7agqaUFoZIDiIyhEp+pOQBCA9GERuO3hVBid7NL2lUuNFlQWu+44NRoNUlNTuSeoa2pq3F49umMwGDB69Gg2rIosy9iyZYuq4y1dEQQBc+fOZcOqrV692u2VvdFoxMSJE7n3eVRVVfXI+LtGo8FvfvMbNqxaXl4e7G6qqjY2NmLbtm1uk2hvI3Yu/87NzeX+u5F/8+93UACSs8Og1V35a4RGBeDOBRl46E+DFB+jJseruvpMGhCKWU8NQGSc8h4LXyorvOR2eEkURYwcOVLVa1dSVFTkdeccHx/PXd7BbrejoKDA63Hi4OBgrzbIffjhh2zoCsHBwV5NpG/YsMFtR8wjOTmZu0CjJEmqiivKsozNmzdj69atbFOvFxsbi+nTp/v1+RW9xZU9qx8JMIhISDFBVCivERKuR8YNEYqP9GERqCppgeTmwKCYxCDc81QWYhL5xto9IUvAyX+5H/M1Go2YMmUKG1btm2++YUMeGzt2LHcHVVtbi4qKCq+HLmbOnMmdJC9duuR2iAWdK4V4x/nRec5zT5g9ezb38uLy8nKX5TV+raGhAc8//7zqare9hSAIGD9+PPdFDPk3v04Q4TGBCI8xeHxGQ+HhC27PeQ4M0mLc7QkICtHhYl17jz9qys04X+R6eAkAUlJSuDvn9vZ2rwv0abVaZGdncw9xnThxQtXeA1cEQcALL7zAhlX7/PPPnS7v/LX77rsPIsdOYnQO5dXW1rJhrwmCgFtvvZUNq7Z161aPFijU1tZi6tSpeOedd1BWVob29na/SBaJiYleJXfSwa+XuQ4eG437/5CN4DD1V1NtZjve/5+DKHOzukmn1yAsygBRYfiqJ0gOCY217bBZXY+L//nPf8bixYvZsConT57EwIED2bBHwsPD8e6773JVkZUkCXl5eZg/f75XnYzJZEJNTQ33Kqrc3FxV5xMXFxdz79J+44038OKLL7Jhr8XFxeHHH39EWloa2+SWJEn43e9+h48++sjt/AtLr9dj+PDhmDRpEjIzMxEdHc21B0cNnU6H1NRUxMXFcc8jSJKEqVOnYvv27WwT8YDfJghRK+CWe5IwbW6q0wquSn7ZXoV1bx336LjR3kKn0yE/Px9ZWVlskyqvv/46Fi1axIY9MnjwYHz++efIzMxkm9xqb2/HokWL8M4777BNHpk2bRrWr1/PNczS3NyMCRMm4OjRo2xTN6Ghoaivr+deKTRkyBAcO3aMDXvt/vvvxwcffMC1y7murg4PPPAAduzYwTapJggCgoKCYDKZuN5/NbRaLYYMGYLFixdzVyu22+2YPHkydu3axTYRD/Cl515A3zn/4ElyMF+yYe/W836ZHNB59ci7Jt9qtbrcGKVWQkIC0tPT2bAqzc3N2LBhAxv22L333stdenv37t2qhn5mzpzJnRyampp65GhRdG4O5F12e/ToUa/LjsuyjJaWFlRXV6OsrKxHHsXFxdi4cSO2bNnCfadpNpupWKAP+G2CCDRqkZimfpWCLAGnD/r3gUE33XQT98SsL8p7o7MGFO9tf2Njo9d1iQwGA+Li4rjfh3379jktcf1rDz30EBtSbfPmzR6N86sVEhKCpKQkrnkRWZZRUlLS6ze/XSbLMsxmM3eCKCoqogThA3zf9F4gKt6IiFj1Y6DmS1Yc21ML8yXvN85dC1qtlrvuEQDs3bvX605Lo9F4VWL8p59+4v7CX5aVleXVJH11dbXLpacmkwkmkwl9+/Zlm1TbsWOHy3+DV1paGgYPHsyVHNvb212W9+5t9Ho9MjIyuH5XAMjPz0dTUxMbJh7y2wSROiRccf+DMxVFzSg43OC3w0vh4eHc5Zftdjt++OEHrxNEcHAwRo4cyYZVkSTJJ+W9R4wYwT1xXFJSgsOHD7PhbsaNG4dx48Zxr9Jqbm5GeXm514lQSXx8PHd574aGBuzevdvjyelrJTExEampqVwJwmq14vjx4z4vkPifSH0P28tkj/CsiNiezRVobfb9Vd3VkpmZiYgI5ZLm7jQ3N6OiokLV0k5Xhg0bhoCAADasii/Ke2s0GkRFRXGvXiorK0N+fj4b7uaWW27BLbfcwp0gfDHOr0Sr1SItLY27hERTUxNOnTrFhnutAQMGID4+ng2rUl9fj9OnT/fIXdx/Gr9MEOExBsQlq5+oO3fiIo7v84+xVyUajQbjxo3jqlqKzvFYb8f+BUHAs88+y4ZV87a8BwBERkZy38HIsoyamhpYrc6HGHU6HWJiYhATE8M1zg8AFRUVPVKeIjw8HA8//DD3/M/x48f9ZshFo9EgLS2Ne6NbZWUlzpw5w4YJB75PGyGEkOueXyaIzBsinR4vyrK2O7Bp9Vmfnih3tRmNRqSlpXFtTJJlGSdPnvR62MNoNGLy5MlsWBW58wwKb8XGxuLmm29mw6rY7XZs3LjR5dxAXFwc+vfvj/79+3ONfaNzpVFgoG/rdomiiGeeeQZDhgxhm1Rbu3at10OMV0tYWBhGjx7NNZwpyzKKi4tRUVHBNhEO6nrZXiZrpPpbz9OHGnDu5JW31rwdwLUQHR3NtXMWnec/lJSUeH3+dEZGBvf4d3t7u9uxfzUiIiK452HsdrvbInUJCQlISkpCUlIS26Ta6NGjMWjQlYdX8dLpdFi0aBEWLlzIvS+jvLzcrzaM3XTTTbjtttu4vqN2ux07d+70+vNOOvhdgtAFiIjrr27+ob3Vjr1bzsNu675yI6ZvEGb+LgOmMD1ErQYaUbhmDzXfgaSkJAwbNowNq2I2m7F//3427LFZs2axIdXOnj3rk7LXvMdrorOmUEtLCxvuxmg0IigoiDsRojOJvfnmm0hNTYXBYIBWq+V6GAwGJCcnY8WKFfjLX/7CfVciSRKee+45l7+7qBWg1WkgaoWuumaCpiOm1Wm6PqOCRoCo7YhpNP/+4Op0Ouj1eq8fISEhmDFjBtatW8e9SKCxsREbN25kw4ST35XaMJq0+O//HYewKNe3n7IMHN9bh8+XnUJTg6UrnpgajEdfGorwWANqK1pRcLABzT44VIiLDOz/rhJN9f9+fSxBEDB//nysWLGCbVKlsLAQI0eO9KqDNhgM+Oqrr7iGmGRZxltvvYWXX37Zq6s6jUaD8+fPo0+fPmyTKs899xyWLl3Khru56aabsGbNGgDgXk56mcViwfr161FQUODxahqdTofs7GxMmTLFq9PRZFnG9u3bMWvWLKd/f0EAbp+ThqBQHapKWnDg+yq0me3IHhmFQWOiIcnApo/PoM1sR+qgcAyZEANRq8GxPbUoONSAoKAgvPLKK9x3N5cFBgZi9OjRGDJkCPdEvMPhwB//+Ee3f2eint8lCEOQFv+9fKzbTXJtLXZ8uaIA//quCnJnWe/YfkG454lMZA5XP0TVk9rNdrz2yG6Xp95pNBqsXLkSv/3tb9kmVb755hvMmDGDDXskKSkJX3zxBYYPH842uWWz2fD0009jxYoVLsf/3YmOjlZVIsOZqKgoNDQ0sOFu0tPT8fHHHwMAxo8fzzb7nbNnz+Lxxx/Hzp07nb73YVEB+POH4xEQKGLvlvP48n8LYGlz4LH/MwwDR0ejvqoN/2/+HkiSjKkPpmDyA8mQJBkrXjqMwsMXkJubi02bNnF36r60efNm3H///S7vlohnrv1f1UPWNofLDvWymvIWHNtT15UcgkJ0yLkvCWlD+cawe0JNeSvazK6vLgMDA5Gbm8uGVfNF7aPRo0dzX1HX1dWhrKzMaQel1uzZs9mQanV1daqWnpaWlqKwsBCFhYVsk99paWnB8uXL8fPPP7t877NGRkEfoIHdJqG+qg1WiwQIQN/0jjI2J/bXwW6XYAzWoU9/E0StALtNQuW5jtL006dP7xXJ4cyZM3j55ZcpOfjYtf/LekiSZJw+5PpKUJaBvVsr0dpsAwBoRAETZvTFqN/EKx4udK0UHmnoSmDOpKSkcG8Yamtrw/r169mwR0RRRFZWFvfk8MmTJ70+l1kQBDz55JNsWLW1a9e67CQvs1qtKCgo8MmJd9eSxWLB6tWr8d5777nc9wGh48REQRDQfNGKssImyJKMvukhCA7XQ5aBklMdCzzCogIQn9wxN1NWeAnmJht0Oh333Jgv1dbW4tVXX8WhQ4fYJuIlv0sQAPCvbyvRfNH5B/98cTMO7awGOifgxk1LxNT/SlE1IXy1WC0OnDvZ5PZUuzvvvJMNqeaLgmWX6xLxrCiRZRnnz59XVRzPldDQUMTFxbFh1bZs2cKGnFq5ciVWrlzpk1VX10JlZSXeeOMN/OEPf3A79xFk0nUcpSsA5iYbqks7SlOMvz0RgiCg5aIV9ZUd5VmCIwIQGtkxrPuv7yohy8DAgQO9qlnlC9XV1XjppZfwz3/+02/KiPgTv0wQdZWt2P5ZieLeBoddxpZPimC1OCAIAkbmxGPGI+kdKzR6kaYGS7fJcyV6vZ7rYJ7LfDG8FB8fj7Fjx7JhVWw2G44dO6bq6t2VnJwc6DnPHmhsbPQoQV24cAEXLlzAwoULvT67+2qSZRlHjhzBs88+i9dff91tcgCAuGQTImI7VkddqG2D+ZIVggBk3NBxt1hx9hKaGiwQNAL6Z4RAbxDhsMsoOtZx0TFy5Eiucyl8Zd++fXjkkUfw4YcfwmbrGC0gvuWXCQIA9m49jx+/KoPk6N75nD3aiMLO40Qzh0dgyn8lw2DkK5vQk6pKWtBU73oYIy4ujvvsBavV6vXwEjoTBG+RQF8McQmCgBkzZnBtmgKAXbt24fz582zYrd27dyMnJ0fV2dXXmizLWLVqFW6//XasX7/e9bDSr0QnGLtOYzyxvx4Ou4zoBCPCYzqSRk25Ga3NNmg0QObwSAgCUF3Wgot17RBF0au9Md6wWCxYtmwZZs2aha1bt/rNBkB/5LcJwtJqx6a8InyTdxY15WZY2hywtDmw79vzsFkd6NM/CLkPpSCyD9/68Z7kcMioKmlByyXXVz3env/gi9r/2dnZ3HWJGhoaUFJSwoY9YjAYEBUVxf0+5Ofne3QH8WuHDh3CxIkT8emnn6KsrAwWi8XruyFfkCQJTU1NOHv2LP7xj38gKysLCxYsQFVVlephFp1eg6h4I3QBImS548IKAAaOiYZG01GBoO58K+w2CTq9iD6de4+O7amDLHfsavfms+EJSZLQ0tKCc+fOYceOHZgwYQKeffZZrsRPPON3y1xZggCERARg0JhoGIxaHNxZhYv1FgweF41Bo6O7Nv70JnabhPyfalF4xPnKGkEQ8NBDD2HSpElskyqnT5/GBx98gObmjtUmvB5//HGMGjWKDaty+vRpvPnmm2zYI1FRUXjssceQmprKNrklyzK++OILj+YglOh0OmRlZWHatGlISEhAQEDANVm5I8sy7HY7WlpacPz4cezYsQOVlZWqhpNYBqMWo6bEo29aMOw2CeuXF8BmlTBmagJSB4fB3GzDvq2VqC5tQXiMAbc9nApBAH7+ugKlBU3o168fFixYwL2AQi2HwwGz2YyCggJ8//33KC4upjuGq8jvE8RlgtDxP5dXBWl1GghX/zusjtxxF8EOj7H0ej33FZrD4YDNZvP6itdgMHBfvTscDtXDHc5oNBro9Xru12Cz2bg6UCWCIEAURWi1Wu7X4y2HwwG73a76TsEZQbj8HREgyzJslo6fp9V1VBaQ5Y4LGVmSoRE7dlUDgN0qQZJkr/8uakmS1PX7evtZJp67bhIEIYQQ3+qt19iEEEKuMUoQhBBCFFGCIIQQoogSBCGEEEWUIAghhCiiBEEIIUQRJQhCCCGKKEEQQghRRAmCEEKIIkoQhBBCFFGCIIQQoogSBCGEEEWUIAghhCiiBEEIIUQRJQhCCCGKKEEQQghRRAmCEEKIIkoQhBBCFFGCIIQQoogSBCGEEEWUIAghhCiiBEEIIUQRJQhCCCGKKEEQQghRRAmCEEKIIkoQhBBCFFGCIIQQoogSBCGEEEWUIAghhCiiBEEIIUQRJQhCCCGKKEEQQghRRAmCEEKIIkoQhBBCFFGCIIQQoogSBCGEEEWUIAghhCiiBEEIIUTR/weHR+ZVVSekggAAAABJRU5ErkJggg==" alt="EMUS">
    <h1>Raptor - Telemetrie en direct</h1>
  </div>
  <div id="statut">Connexion...</div>

  <div class="graphiques">
    <div class="carte-graph">
      <h2>Temperatures moteur / onduleur</h2>
      <canvas id="graphTemp"></canvas>
      <div class="legende-graph" id="graphTempLegende"></div>
      <div class="tooltip-graph" id="graphTempTooltip"></div>
    </div>
    <div class="carte-graph">
      <h2>Vitesse / RPM moteur</h2>
      <canvas id="graphSpeed"></canvas>
      <div class="legende-graph" id="graphSpeedLegende"></div>
      <div class="tooltip-graph" id="graphSpeedTooltip"></div>
    </div>
    <div class="carte-graph">
      <h2>Tension batterie</h2>
      <canvas id="graphBat"></canvas>
      <div class="legende-graph" id="graphBatLegende"></div>
      <div class="tooltip-graph" id="graphBatTooltip"></div>
    </div>
    <div class="carte-graph">
      <h2>E-STOP</h2>
      <canvas id="graphEstop"></canvas>
      <div class="legende-graph" id="graphEstopLegende"></div>
      <div class="tooltip-graph" id="graphEstopTooltip"></div>
    </div>
  </div>

  <input id="filtre" placeholder="filtrer par nom...">
  <table>
    <thead><tr><th>Capteur</th><th>Valeur</th></tr></thead>
    <tbody id="corps_table"></tbody>
  </table>

<script>
let conversions = {};
let dernieresValeurs = {};   // derniere valeur brute vue par capteur, pour detecter les changements

async function chargerConversions() {
  try {
    const rep = await fetch("/api/conversions");
    conversions = await rep.json();
  } catch (e) {
    // pas grave : on retombera juste sur l'affichage brut partout
  }
}

function formaterValeur(cle, brut) {
  const conv = conversions[cle];
  if (!conv) return brut;
  const [scale, offset, unite] = conv;
  const physique = brut * scale + offset;
  return `${physique.toFixed(2)}${unite ? " " + unite : ""} (brut ${brut})`;
}

function valeurPhysique(cle, brut) {
  const conv = conversions[cle];
  if (!conv) return { valeur: brut, unite: "" };
  const [scale, offset, unite] = conv;
  return { valeur: brut * scale + offset, unite };
}

// ============================================================
//  GRAPHIQUES DE TENDANCE (temperature, vitesse/RPM)
//  Fenetre glissante de 60s, tenue cote navigateur (pas de
//  persistance serveur). Un point par cycle de maj() (200ms).
// ============================================================
const FENETRE_MS = 60000;
const COULEURS_GRAPH = ["#3987e5", "#d95926"];   // palette categorique validee, mode sombre

const graphiques = [
  {
    id: "graphTemp",
    cles: [
      { cle: "motor_temp", nom: "Moteur" },
      { cle: "inverter_temp", nom: "Onduleur" },
    ],
    historique: [],
    geom: null,
  },
  {
    id: "graphSpeed",
    cles: [
      { cle: "speed_ref", nom: "Consigne" },
      { cle: "speed_measure", nom: "Mesuree" },
    ],
    historique: [],
    geom: null,
  },
  {
    id: "graphBat",
    cles: [
      { cle: "v_bat", nom: "Tension" },
    ],
    historique: [],
    geom: null,
  },
  {
    id: "graphEstop",
    cles: [
      { cle: "e_stop", nom: "E-STOP" },
    ],
    historique: [],
    geom: null,
  },
];

function redimensionnerCanvas(canvas) {
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  canvas.width = Math.round(rect.width * dpr);
  canvas.height = Math.round(rect.height * dpr);
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ctx, largeur: rect.width, hauteur: rect.height };
}

function dessinerGraphique(cfg) {
  const canvas = document.getElementById(cfg.id);
  if (!canvas) return;
  const { ctx, largeur, hauteur } = redimensionnerCanvas(canvas);
  ctx.clearRect(0, 0, largeur, hauteur);

  const PAD_G = 34, PAD_D = 8, PAD_H = 8, PAD_B = 16;
  const zoneL = largeur - PAD_G - PAD_D;
  const zoneH = hauteur - PAD_H - PAD_B;

  const maintenant = Date.now();
  const tMin = maintenant - FENETRE_MS;

  ctx.font = "10px system-ui, sans-serif";
  ctx.fillStyle = "#898781";

  if (cfg.historique.length < 2) {
    ctx.textBaseline = "middle";
    ctx.fillText("En attente de donnees...", PAD_G, PAD_H + zoneH / 2);
    cfg.geom = null;
    return;
  }

  let vMin = Infinity, vMax = -Infinity;
  for (const pt of cfg.historique) {
    for (const s of cfg.cles) {
      const v = pt[s.cle];
      if (v === undefined) continue;
      if (v < vMin) vMin = v;
      if (v > vMax) vMax = v;
    }
  }
  if (!isFinite(vMin)) { cfg.geom = null; return; }
  if (vMin === vMax) { vMin -= 1; vMax += 1; }
  const marge = (vMax - vMin) * 0.1;
  vMin -= marge; vMax += marge;

  const xPour = (t) => PAD_G + ((t - tMin) / FENETRE_MS) * zoneL;
  const yPour = (v) => PAD_H + zoneH - ((v - vMin) / (vMax - vMin)) * zoneH;

  // Gridlines horizontales (hairline, recessives) + ticks Y
  ctx.strokeStyle = "#2c2c2a";
  ctx.lineWidth = 1;
  ctx.textBaseline = "middle";
  const nbLignes = 3;
  for (let i = 0; i <= nbLignes; i++) {
    const v = vMin + (vMax - vMin) * (i / nbLignes);
    const y = yPour(v);
    ctx.beginPath();
    ctx.moveTo(PAD_G, y);
    ctx.lineTo(PAD_G + zoneL, y);
    ctx.stroke();
    ctx.fillStyle = "#898781";
    ctx.fillText(v.toFixed(1), 2, y);
  }

  // Axe X : bornes de la fenetre glissante
  ctx.textBaseline = "alphabetic";
  ctx.fillText("-60s", PAD_G, hauteur - 3);
  ctx.fillText("maintenant", PAD_G + zoneL - 44, hauteur - 3);

  // Lignes de donnees + marqueur de fin (>=8px, anneau surface)
  cfg.cles.forEach((s, i) => {
    const couleur = COULEURS_GRAPH[i % COULEURS_GRAPH.length];
    ctx.strokeStyle = couleur;
    ctx.lineWidth = 2;
    ctx.lineJoin = "round";
    ctx.lineCap = "round";
    ctx.beginPath();
    let dernierPoint = null;
    let tracee = false;
    for (const pt of cfg.historique) {
      const v = pt[s.cle];
      if (v === undefined) continue;
      const x = xPour(pt.t), y = yPour(v);
      if (!tracee) { ctx.moveTo(x, y); tracee = true; }
      else ctx.lineTo(x, y);
      dernierPoint = { x, y };
    }
    if (tracee) ctx.stroke();

    if (dernierPoint) {
      ctx.beginPath();
      ctx.arc(dernierPoint.x, dernierPoint.y, 5, 0, Math.PI * 2);
      ctx.fillStyle = couleur;
      ctx.fill();
      ctx.lineWidth = 2;
      ctx.strokeStyle = "#1a1a19";
      ctx.stroke();
    }
  });

  cfg.geom = { PAD_G, zoneL, tMin };

  // Legende : trait de couleur (pas de pastille pleine) + valeur actuelle
  const legende = document.getElementById(cfg.id + "Legende");
  if (legende) {
    legende.innerHTML = "";
    cfg.cles.forEach((s, i) => {
      let dernier;
      for (let j = cfg.historique.length - 1; j >= 0; j--) {
        if (cfg.historique[j][s.cle] !== undefined) { dernier = cfg.historique[j][s.cle]; break; }
      }
      const couleur = COULEURS_GRAPH[i % COULEURS_GRAPH.length];
      const unite = conversions[s.cle] ? conversions[s.cle][2] : "";

      const span = document.createElement("span");
      span.className = "cle";
      const trait = document.createElement("span");
      trait.className = "trait";
      trait.style.background = couleur;
      const texte = document.createElement("span");
      texte.textContent = `${s.nom}: ${dernier !== undefined ? dernier.toFixed(1) : "--"}${unite ? " " + unite : ""}`;
      span.appendChild(trait);
      span.appendChild(texte);
      legende.appendChild(span);
    });
  }
}

function attacherSurvolGraphique(cfg) {
  const canvas = document.getElementById(cfg.id);
  const infobulle = document.getElementById(cfg.id + "Tooltip");
  if (!canvas || !infobulle) return;

  canvas.addEventListener("mousemove", (e) => {
    if (!cfg.geom || cfg.historique.length === 0) {
      infobulle.style.display = "none";
      return;
    }
    const rect = canvas.getBoundingClientRect();
    const xSouris = e.clientX - rect.left;
    const tSouris = cfg.geom.tMin + ((xSouris - cfg.geom.PAD_G) / cfg.geom.zoneL) * FENETRE_MS;

    let plusProche = null, meilleurEcart = Infinity;
    for (const pt of cfg.historique) {
      const ecart = Math.abs(pt.t - tSouris);
      if (ecart < meilleurEcart) { meilleurEcart = ecart; plusProche = pt; }
    }
    if (!plusProche) { infobulle.style.display = "none"; return; }

    infobulle.innerHTML = "";
    cfg.cles.forEach((s) => {
      if (plusProche[s.cle] === undefined) return;
      const unite = conversions[s.cle] ? conversions[s.cle][2] : "";
      const ligne = document.createElement("div");
      const nom = document.createTextNode(s.nom + ": ");
      const val = document.createElement("strong");
      val.textContent = plusProche[s.cle].toFixed(2) + (unite ? " " + unite : "");
      ligne.appendChild(nom);
      ligne.appendChild(val);
      infobulle.appendChild(ligne);
    });
    infobulle.style.display = "block";
    infobulle.style.left = Math.min(xSouris + 10, canvas.clientWidth - 90) + "px";
    infobulle.style.top = "4px";
  });

  canvas.addEventListener("mouseleave", () => {
    infobulle.style.display = "none";
  });
}

async function maj() {
  try {
    const rep = await fetch("/api/data");
    const etat = await rep.json();
    const statut = document.getElementById("statut");
    if (etat.connecte) {
      statut.innerHTML = `<span class="dot ok"></span>Port ${etat.port} - derniere trame: ${etat.derniere_maj || "aucune"}`;
    } else {
      statut.innerHTML = `<span class="dot ko"></span>Deconnecte (${etat.erreur || "en attente du port serie"})`;
    }

    // Detection de changement, TOUT capteur confondu (pas juste ceux avec
    // un graphique dedie) : sert a surligner brievement la ligne du tableau
    // pour qu'un changement rapide (ex: v_bat qui chute) saute aux yeux
    // meme sans historique/graphique pour ce capteur precis.
    const cellesChangees = new Set();
    for (const [cle, valeur] of Object.entries(etat.donnees)) {
      if (dernieresValeurs[cle] !== undefined && dernieresValeurs[cle] !== valeur) {
        cellesChangees.add(cle);
      }
      dernieresValeurs[cle] = valeur;
    }

    const filtre = document.getElementById("filtre").value.toLowerCase();
    const corps = document.getElementById("corps_table");
    corps.innerHTML = "";
    for (const [cle, valeur] of Object.entries(etat.donnees)) {
      if (filtre && !cle.toLowerCase().includes(filtre)) continue;
      const tr = document.createElement("tr");
      if (cellesChangees.has(cle)) tr.className = "changee";
      tr.innerHTML = `<td>${cle}</td><td>${formaterValeur(cle, valeur)}</td>`;
      corps.appendChild(tr);
    }

    // Graphiques de tendance : un point par cycle, fenetre glissante de 60s.
    graphiques.forEach((cfg) => {
      const point = { t: Date.now() };
      let quelconque = false;
      cfg.cles.forEach((s) => {
        if (etat.donnees[s.cle] !== undefined) {
          point[s.cle] = valeurPhysique(s.cle, etat.donnees[s.cle]).valeur;
          quelconque = true;
        }
      });
      if (quelconque) {
        cfg.historique.push(point);
        const limite = Date.now() - FENETRE_MS;
        while (cfg.historique.length && cfg.historique[0].t < limite) cfg.historique.shift();
      }
      dessinerGraphique(cfg);
    });
  } catch (e) {
    document.getElementById("statut").textContent = "Erreur de connexion au serveur Flask";
  }
}
chargerConversions();
graphiques.forEach(attacherSurvolGraphique);
window.addEventListener("resize", () => graphiques.forEach(dessinerGraphique));
setInterval(maj, 200);
maj();
</script>
</body>
</html>
"""

if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else PORT_PAR_DEFAUT

    print("Ports serie disponibles :")
    for p in serial.tools.list_ports.comports():
        print(f"  {p.device} - {p.description}")
    print(f"Utilisation du port : {port} (change avec: python app.py COMx)")

    threading.Thread(target=lire_serie, args=(port,), daemon=True).start()
    app.run(host="0.0.0.0", port=5000, debug=False)
