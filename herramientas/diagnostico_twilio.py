#!/usr/bin/env python3
"""
diagnostico_twilio.py — verifica la configuración de Twilio de Ana POR SEPARADO,
sin arrancar ROS ni el LLM. Sirve para aislar dónde falla la llamada de emergencia.

Lee las credenciales SOLO de variables de entorno (las mismas que usa
gestor_emergencia.py): TWILIO_SID, TWILIO_TOKEN, TWILIO_FROM, TWILIO_TO.
Opcionalmente TWILIO_BRIDGE_URL y EMERGENCY_WAYPOINT. Nunca imprime los secretos
en claro (los enmascara).

Uso:
  python3 herramientas/diagnostico_twilio.py            # validación SOLO LECTURA
  python3 herramientas/diagnostico_twilio.py --twiml     # + imprime el TwiML
  python3 herramientas/diagnostico_twilio.py --llamar    # COLOCA UNA LLAMADA REAL

La validación read-only autentica, comprueba el tipo de cuenta, que el número
FROM pertenezca a la cuenta, que el TO esté verificado (obligatorio en cuentas
trial) y el saldo. No llama a nadie salvo que pases --llamar.
"""

import argparse
import json
import os
import sys
from xml.sax.saxutils import escape, quoteattr

PERFIL_PATH = os.path.join(
    os.path.expanduser('~'), 'asistente_turtlebot4-main', 'perfil_residente.json')


def mask(s):
    """Enmascara un secreto para imprimirlo sin exponerlo."""
    if not s:
        return '(VACÍO)'
    return (s[:4] + '…' + s[-2:]) if len(s) > 6 else '(corto)'


def construir_twiml():
    """Replica el TwiML de gestor_emergencia._construir_twiml (sin depender del nodo)."""
    bridge = os.environ.get('TWILIO_BRIDGE_URL', '').strip()
    if bridge:
        return (f'<Response><Connect>'
                f'<Stream url={quoteattr(bridge)}/>'
                f'</Connect></Response>')
    perfil = {}
    try:
        with open(PERFIL_PATH, 'r', encoding='utf-8') as f:
            perfil = json.load(f)
    except (OSError, ValueError):
        pass
    nombre = perfil.get('nombre', '')
    direccion = perfil.get('direccion', '')
    partes = ['Soy Ana, la asistente robótica']
    if nombre:
        partes.append(f' de {nombre}')
    partes.append('. Puede haber una emergencia')
    if direccion:
        partes.append(f' en {direccion}')
    partes.append('. Por favor, acuda lo antes posible.')
    mensaje = ''.join(partes)
    return (f'<Response>'
            f'<Say voice="Polly.Conchita" language="es-ES">{escape(mensaje)}</Say>'
            f'<Hangup/>'
            f'</Response>')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--twiml', action='store_true',
                    help='imprime el TwiML que se enviaría')
    ap.add_argument('--llamar', action='store_true',
                    help='COLOCA UNA LLAMADA REAL al TWILIO_TO (acción real)')
    args = ap.parse_args()

    sid = os.environ.get('TWILIO_SID', '')
    tok = os.environ.get('TWILIO_TOKEN', '')
    frm = os.environ.get('TWILIO_FROM', '')
    to = os.environ.get('TWILIO_TO', '')

    print('=== Variables de entorno ===')
    print('  TWILIO_SID  :', mask(sid), '| empieza por AC?', sid.startswith('AC'),
          '| len', len(sid))
    print('  TWILIO_TOKEN:', 'SET' if tok else 'VACÍO', '| len', len(tok))
    print('  TWILIO_FROM :', frm, '| E.164 (+...)?', frm.startswith('+'))
    print('  TWILIO_TO   :', to, '| E.164 (+...)?', to.startswith('+'))
    print('  BRIDGE_URL  :', os.environ.get('TWILIO_BRIDGE_URL', '') or '(vacío → mensaje hablado)')

    if not all([sid, tok, frm, to]):
        print('\nFALTAN credenciales. Exporta TWILIO_SID/TOKEN/FROM/TO '
              '(p.ej. `source secrets.env`).')
        return 1

    try:
        from twilio.rest import Client
    except ImportError:
        print('\nTwilio no instalado: pip install twilio')
        return 1

    client = Client(sid, tok)

    print('\n=== Validación (solo lectura) ===')
    try:
        acc = client.api.accounts(sid).fetch()
        print('  AUTH OK → cuenta:', acc.friendly_name,
              '| status:', acc.status, '| tipo:', getattr(acc, 'type', '?'))
    except Exception as e:
        print('  FALLO de autenticación:', type(e).__name__, str(e)[:200])
        return 1

    try:
        nums = [n.phone_number for n in client.incoming_phone_numbers.list(limit=20)]
        print('  Números Twilio de la cuenta:', nums)
        print('  ¿FROM pertenece a la cuenta? →', frm in nums)
    except Exception as e:
        print('  No pude listar números:', str(e)[:120])

    try:
        oci = [o.phone_number for o in client.outgoing_caller_ids.list(limit=20)]
        verificado = to in oci
        print('  Caller IDs verificados:', oci)
        print('  ¿TO verificado? →', verificado,
              '(obligatorio en cuentas Trial)')
        if getattr(acc, 'type', '') == 'Trial' and not verificado:
            print('  AVISO: cuenta Trial y TO NO verificado → la llamada fallará '
                  '(error 21219). Verifica el número en console.twilio.com.')
    except Exception as e:
        print('  No pude comprobar caller IDs:', str(e)[:120])

    try:
        bal = client.api.accounts(sid).balance.fetch()
        print('  Saldo:', bal.balance, bal.currency)
    except Exception as e:
        print('  Saldo no consultable:', str(e)[:120])

    twiml = construir_twiml()
    if args.twiml or args.llamar:
        print('\n=== TwiML que se enviaría ===')
        print(' ', twiml)

    if args.llamar:
        print(f'\n=== LLAMADA REAL → {to} ===')
        resp = input('Esto hará SONAR un teléfono de verdad. ¿Continuar? [escribe SI]: ')
        if resp.strip().upper() != 'SI':
            print('Cancelado.')
            return 0
        try:
            call = client.calls.create(to=to, from_=frm, twiml=twiml)
            print('  Llamada creada OK. SID:', call.sid, '| status:', call.status)
            print('  (consulta el estado final en console.twilio.com → Monitor → Logs → Calls)')
        except Exception as e:
            print('  FALLO al crear la llamada:', type(e).__name__, str(e)[:300])
            return 1

    print('\nListo.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
