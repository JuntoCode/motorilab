// Families and engines shown on the site. `load` is present only for engines that are ready.
export const FAMILIES = [
  {
    id: 'terra',
    name: 'Terra',
    desc: 'Auto, moto e mezzi su strada: dal classico motore a scoppio all’elettrico.',
    image: 'assets/img/family-terra.png',
    engines: [
      { id: 'petrol4t', short: 'Benzina 4 tempi', vehicle: 'Automobile', load: () => import('./engines/petrol4t.js') },
      { id: 'diesel', short: 'Diesel common rail', vehicle: 'Auto, camion', load: () => import('./engines/diesel.js') },
      { id: 'moto4t', short: 'Moto 4 tempi', vehicle: 'Moto sportiva e stradale', load: () => import('./engines/moto4t.js') },
      { id: 'twoStroke', short: 'Moto 2 tempi', vehicle: 'Scooter, moto da cross', load: () => import('./engines/two-stroke.js') },
      { id: 'wankel', short: 'Rotativo Wankel', vehicle: 'Auto sportive', load: () => import('./engines/wankel.js') },
      { id: 'evMotor', short: 'Elettrico a magneti', vehicle: 'Auto elettrica', load: () => import('./engines/ev-motor.js') },
      { id: 'hybrid', short: 'Ibrido', vehicle: 'Auto ibrida', load: () => import('./engines/hybrid.js') },
    ],
  },
  {
    id: 'aria',
    name: 'Aria',
    desc: 'Aerei ed elicotteri: turbine che spingono tonnellate di metallo nel cielo.',
    image: 'assets/img/family-aria.png',
    engines: [
      { id: 'turbofan', short: 'Turbofan', vehicle: 'Aereo di linea', load: () => import('./engines/turbofan.js') },
      { id: 'turboprop', short: 'Turboelica', vehicle: 'Aereo regionale', load: () => import('./engines/turboprop.js') },
      { id: 'aeroPiston', short: 'Pistoni aeronautico', vehicle: 'Aereo da turismo', load: () => import('./engines/aero-piston.js') },
      { id: 'turboshaft', short: 'Turboalbero', vehicle: 'Elicottero', load: () => import('./engines/turboshaft.js') },
    ],
  },
  {
    id: 'spazio',
    name: 'Spazio',
    desc: 'Razzi e sonde: come si spinge un veicolo dove non c’è aria.',
    image: 'assets/img/family-spazio.png',
    engines: [
      { id: 'rocket', short: 'Razzo a liquido', vehicle: 'Lanciatore spaziale', load: () => import('./engines/rocket.js') },
      { id: 'solidRocket', short: 'Razzo a solido', vehicle: 'Booster, Vega', load: () => import('./engines/solid-rocket.js') },
      { id: 'ion', short: 'Propulsore ionico', vehicle: 'Sonde e satelliti', load: () => import('./engines/ion.js') },
    ],
  },
  {
    id: 'mare',
    name: 'Mare e rotaia',
    desc: 'Treni e navi: i motori più grandi ed efficienti del mondo.',
    image: 'assets/img/family-mare.png',
    engines: [
      {
        id: 'electricTrain',
        short: 'Elettrico ferroviario',
        vehicle: 'Treno elettrico',
        load: () => import('./engines/electric-train.js'),
      },
      { id: 'marineDiesel', short: 'Diesel navale 2 tempi', vehicle: 'Nave portacontainer', load: () => import('./engines/marine-diesel.js') },
      { id: 'dieselElectric', short: 'Diesel-elettrico', vehicle: 'Locomotiva', load: () => import('./engines/diesel-electric.js') },
    ],
  },
];

export const READY = FAMILIES.flatMap((f) => f.engines.filter((e) => e.load).map((e) => ({ ...e, family: f.id })));

const cache = new Map();
export async function loadEngine(id) {
  if (cache.has(id)) return cache.get(id);
  const entry = READY.find((e) => e.id === id);
  if (!entry) throw new Error(`Motore sconosciuto: ${id}`);
  const def = (await entry.load()).default;
  cache.set(id, def);
  return def;
}
