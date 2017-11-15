﻿// <auto-generated />
using Microsoft.EntityFrameworkCore;
using Microsoft.EntityFrameworkCore.Infrastructure;
using Microsoft.EntityFrameworkCore.Metadata;
using Microsoft.EntityFrameworkCore.Migrations;
using Microsoft.EntityFrameworkCore.Storage;
using Microsoft.EntityFrameworkCore.Storage.Internal;
using System;
using Willy.Web.Contexts;

namespace Willy.Web.Migrations
{
    [DbContext(typeof(WillyContext))]
    partial class WillyContextModelSnapshot : ModelSnapshot
    {
        protected override void BuildModel(ModelBuilder modelBuilder)
        {
#pragma warning disable 612, 618
            modelBuilder
                .HasAnnotation("Npgsql:ValueGenerationStrategy", NpgsqlValueGenerationStrategy.SerialColumn)
                .HasAnnotation("ProductVersion", "2.0.0-rtm-26452");

            modelBuilder.Entity("Willy.Web.ControlPanel.Entities.Command", b =>
                {
                    b.Property<int>("CommandId")
                        .ValueGeneratedOnAdd();

                    b.Property<string>("Content");

                    b.Property<string>("Description");

                    b.Property<string>("Name");

                    b.HasKey("CommandId");

                    b.ToTable("Commands");
                });
#pragma warning restore 612, 618
        }
    }
}
