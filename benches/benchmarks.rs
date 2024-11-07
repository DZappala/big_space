#![allow(clippy::type_complexity)]

use bevy::prelude::*;
use big_space::{
    spatial_hash::{SpatialHashMap, SpatialHashPlugin},
    *,
};
use criterion::{black_box, criterion_group, criterion_main, Criterion};
use std::iter::repeat_with;
use turborand::prelude::*;

criterion_group!(
    benches,
    global_transform,
    spatial_hashing,
    hash_filtering,
    deep_hierarchy,
    wide_hierarchy,
    vs_bevy,
);
criterion_main!(benches);

#[allow(clippy::unit_arg)]
fn global_transform(c: &mut Criterion) {
    let mut group = c.benchmark_group("propagation");
    group.bench_function("global_transform", |b| {
        let frame = ReferenceFrame::default();
        let local_cell = GridCell { x: 1, y: 1, z: 1 };
        let local_transform = Transform::from_xyz(9.0, 200.0, 500.0);
        b.iter(|| {
            black_box(frame.global_transform(&local_cell, &local_transform));
        });
    });
}

#[allow(clippy::unit_arg)]
fn deep_hierarchy(c: &mut Criterion) {
    let mut group = c.benchmark_group("deep_hierarchy");

    /// Total number of entities to spawn
    const N_SPAWN: usize = 100;

    fn setup(mut commands: Commands) {
        commands.spawn_big_space(ReferenceFrame::<i32>::new(10000.0, 0.0), |root| {
            let mut parent = root.spawn_frame_default(()).id();
            for _ in 0..N_SPAWN {
                let child = root
                    .commands()
                    .spawn(BigReferenceFrameBundle::<i32>::default())
                    .id();
                root.commands().entity(parent).add_child(child);
                parent = child;
            }
            root.spawn_spatial(FloatingOrigin);
        });
    }

    fn translate(mut transforms: Query<&mut Transform>) {
        transforms.iter_mut().for_each(|mut transform| {
            transform.translation += Vec3::ONE;
        })
    }

    let mut app = App::new();
    app.add_plugins((
        SpatialHashPlugin::<i32>::default(),
        BigSpacePlugin::<i32>::default(),
    ))
    .add_systems(Startup, setup)
    .add_systems(Update, translate)
    .update();

    group.bench_function("Baseline", |b| {
        b.iter(|| {
            black_box(app.update());
        });
    });
}

#[allow(clippy::unit_arg)]
fn wide_hierarchy(c: &mut Criterion) {
    let mut group = c.benchmark_group("wide_hierarchy");

    /// Total number of entities to spawn
    const N_SPAWN: usize = 100_000;

    fn setup(mut commands: Commands) {
        commands.spawn_big_space(ReferenceFrame::<i32>::new(10000.0, 0.0), |root| {
            for _ in 0..N_SPAWN {
                root.spawn_spatial(());
            }
            root.spawn_spatial(FloatingOrigin);
        });
    }

    fn translate(mut transforms: Query<&mut Transform>) {
        transforms.iter_mut().for_each(|mut transform| {
            transform.translation += Vec3::ONE;
        })
    }

    let mut app = App::new();
    app.add_plugins((
        SpatialHashPlugin::<i32>::default(),
        BigSpacePlugin::<i32>::default(),
    ))
    .add_systems(Startup, setup)
    .add_systems(Update, translate)
    .update();

    group.bench_function("Baseline", |b| {
        b.iter(|| {
            black_box(app.update());
        });
    });
}

#[allow(clippy::unit_arg)]
fn spatial_hashing(c: &mut Criterion) {
    let mut group = c.benchmark_group("spatial_hashing");

    const HALF_WIDTH: i32 = 100;
    /// Total number of entities to spawn
    const N_SPAWN: usize = 10_000;
    /// Number of entities that move into a different cell each update
    const N_MOVE: usize = 1_000;

    fn setup(mut commands: Commands) {
        commands.spawn_big_space(ReferenceFrame::<i32>::new(1.0, 0.0), |root| {
            let rng = Rng::with_seed(342525);
            let values: Vec<_> = repeat_with(|| {
                [
                    rng.i32(-HALF_WIDTH..=HALF_WIDTH),
                    rng.i32(-HALF_WIDTH..=HALF_WIDTH),
                    rng.i32(-HALF_WIDTH..=HALF_WIDTH),
                ]
            })
            .take(N_SPAWN)
            .collect();

            root.with_children(|root| {
                for pos in values {
                    root.spawn(BigSpatialBundle {
                        cell: GridCell::new(pos[0], pos[1], pos[2]),
                        ..Default::default()
                    });
                }
            });
        });
    }

    fn translate(mut cells: Query<&mut GridCell<i32>>) {
        cells.iter_mut().take(N_MOVE).for_each(|mut cell| {
            *cell += GridCell::ONE;
        })
    }

    let mut app = App::new();
    app.add_plugins(SpatialHashPlugin::<i32>::default())
        .add_systems(Startup, setup)
        .update();

    group.bench_function("Baseline", |b| {
        b.iter(|| {
            black_box(app.update());
        });
    });

    app.add_systems(Update, translate).update();
    group.bench_function("Translation and rehashing", |b| {
        b.iter(|| {
            black_box(app.update());
        });
    });

    let map = app.world().resource::<SpatialHashMap<i32>>();
    let first = map
        .iter()
        .find(|(_, entry)| !entry.entities.is_empty())
        .unwrap();
    group.bench_function("SpatialHashMap::get", |b| {
        b.iter(|| {
            black_box(map.get(first.0).unwrap());
        });
    });

    let ent = *first.1.entities.iter().next().unwrap();
    group.bench_function("Find entity", |b| {
        b.iter(|| {
            black_box(map.get(first.0).map(|entry| entry.entities.get(&ent)));
        });
    });

    let parent = app
        .world_mut()
        .query::<&Parent>()
        .get(app.world(), ent)
        .unwrap();
    let map = app.world().resource::<SpatialHashMap<i32>>();

    group.bench_function("Neighbors radius: 1", |b| {
        b.iter(|| {
            black_box(map.neighbors(1, parent, GridCell::new(0, 0, 0)).count());
        });
    });

    group.bench_function("Neighbors radius: 4", |b| {
        b.iter(|| {
            black_box(map.neighbors(4, parent, GridCell::new(0, 0, 0)).count());
        });
    });

    group.bench_function(format!("Neighbors radius: {}", HALF_WIDTH), |b| {
        b.iter(|| {
            black_box(
                map.neighbors(HALF_WIDTH as u8, parent, GridCell::new(0, 0, 0))
                    .count(),
            );
        });
    });

    let flood = || map.neighbors_contiguous(1, parent, GridCell::ZERO).count();
    group.bench_function("Neighbors flood 1", |b| {
        b.iter(|| black_box(flood()));
    });
}

#[allow(clippy::unit_arg)]
fn hash_filtering(c: &mut Criterion) {
    let mut group = c.benchmark_group("hash_filtering");

    const N_ENTITIES: usize = 100_000;
    const N_PLAYERS: usize = 100;
    const N_MOVE: usize = 1_000;
    const HALF_WIDTH: i32 = 100;

    #[derive(Component)]
    struct Player;

    fn setup(mut commands: Commands) {
        let rng = Rng::with_seed(342525);
        let values: Vec<_> = repeat_with(|| {
            [
                rng.i32(-HALF_WIDTH..=HALF_WIDTH),
                rng.i32(-HALF_WIDTH..=HALF_WIDTH),
                rng.i32(-HALF_WIDTH..=HALF_WIDTH),
            ]
        })
        .take(N_ENTITIES)
        .collect();

        commands.spawn_big_space(ReferenceFrame::<i32>::default(), |root| {
            root.with_children(|root| {
                for (i, pos) in values.iter().enumerate() {
                    let mut cmd = root.spawn(BigSpatialBundle {
                        cell: GridCell::new(pos[0], pos[1], pos[2]),
                        ..Default::default()
                    });
                    if i < N_PLAYERS {
                        cmd.insert(Player);
                    }
                }
            });
        });
    }

    fn translate(mut cells: Query<&mut GridCell<i32>>) {
        cells.iter_mut().take(N_MOVE).for_each(|mut cell| {
            *cell += IVec3::ONE;
        });
    }

    let mut app = App::new();
    app.add_systems(Startup, setup)
        .add_systems(Update, translate)
        .update();
    app.update();
    app.add_plugins((SpatialHashPlugin::<i32>::default(),));
    group.bench_function("No Filter Plugin", |b| {
        b.iter(|| {
            black_box(app.update());
        });
    });

    let mut app = App::new();
    app.add_systems(Startup, setup)
        .add_systems(Update, translate)
        .update();
    app.update();
    app.add_plugins((SpatialHashPlugin::<i32, With<Player>>::default(),));
    group.bench_function("With Player Plugin", |b| {
        b.iter(|| {
            black_box(app.update());
        });
    });

    let mut app = App::new();
    app.add_systems(Startup, setup)
        .add_systems(Update, translate)
        .update();
    app.update();
    app.add_plugins((SpatialHashPlugin::<i32, Without<Player>>::default(),));
    group.bench_function("Without Player Plugin", |b| {
        b.iter(|| {
            black_box(app.update());
        });
    });

    let mut app = App::new();
    app.add_systems(Startup, setup)
        .add_systems(Update, translate)
        .update();
    app.update();
    app.add_plugins((SpatialHashPlugin::<i32>::default(),))
        .add_plugins((SpatialHashPlugin::<i32, With<Player>>::default(),))
        .add_plugins((SpatialHashPlugin::<i32, Without<Player>>::default(),));
    group.bench_function("All Plugins", |b| {
        b.iter(|| {
            black_box(app.update());
        });
    });
}

#[allow(clippy::unit_arg)]
fn vs_bevy(c: &mut Criterion) {
    let mut group = c.benchmark_group("transform_prop");

    use bevy::prelude::*;
    use big_space::BigSpacePlugin;

    const N_ENTITIES: usize = 100_000;

    fn setup_bevy(mut commands: Commands) {
        commands
            .spawn(SpatialBundle::default())
            .with_children(|builder| {
                for _ in 0..N_ENTITIES {
                    builder.spawn(SpatialBundle::default());
                }
            });
    }

    fn setup_big(mut commands: Commands) {
        commands
            .spawn(BigSpaceRootBundle::<i32>::default())
            .with_children(|builder| {
                for _ in 0..N_ENTITIES {
                    builder.spawn((SpatialBundle::default(), GridCell::<i32>::default()));
                }
            });
    }

    fn translate(mut transforms: Query<&mut Transform, With<Children>>) {
        transforms.iter_mut().for_each(|mut transform| {
            transform.translation += 1.0;
        });
    }

    let mut app = App::new();
    app.add_plugins((MinimalPlugins, TransformPlugin))
        .add_systems(Startup, setup_bevy)
        .update();

    group.bench_function("Bevy Propagation Static", |b| {
        b.iter(|| {
            black_box(app.update());
        });
    });

    let mut app = App::new();
    app.add_plugins((MinimalPlugins, BigSpacePlugin::<i32>::default()))
        .add_systems(Startup, setup_big)
        .update();

    group.bench_function("Big Space Propagation Static", |b| {
        b.iter(|| {
            black_box(app.update());
        });
    });

    let mut app = App::new();
    app.add_plugins((MinimalPlugins, TransformPlugin))
        .add_systems(Startup, setup_bevy)
        .add_systems(Update, translate)
        .update();

    group.bench_function("Bevy Propagation", |b| {
        b.iter(|| {
            black_box(app.update());
        });
    });

    let mut app = App::new();
    app.add_plugins((MinimalPlugins, BigSpacePlugin::<i32>::default()))
        .add_systems(Startup, setup_big)
        .add_systems(Update, translate)
        .update();

    group.bench_function("Big Space Propagation", |b| {
        b.iter(|| {
            black_box(app.update());
        });
    });
}
